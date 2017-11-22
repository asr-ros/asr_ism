/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Global includes
#include <string>
#include <vector>

//Pkg includes
#include <ros/ros.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/viz_helper.hpp>
#include <ISM/common_type/Pose.hpp>
#include <ISM/common_type/Track.hpp>
#include <ISM/common_type/Tracks.hpp>

#include <ISM/utility/GeometryHelper.hpp>
#include <ISM/utility/Util.hpp>

#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

#include <iostream>
#include <fstream>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>



struct termios originalSettings, modifiedSettings;
bool keyPressed;

#define KEY_NEXT_OBJECT 0x77 //w
#define KEY_PREV_OBJECT 0x73 //s
#define KEY_NEXT_POSE 0x64 //d
#define KEY_PREV_POSE 0x61 //a
#define KEY_SWITCH_EDIT 0x65 //e
#define KEY_MARK 0x6D //m
#define KEY_SAVE 0x6F //o
#define KEY_SHOW 0x70 //p

class ObjectConfigurationGenerator
{

public:

    ObjectConfigurationGenerator() : nh("~")
    {
        std::vector<std::string> patternNames;
        std::string configFilePath;
        getNodeParameters(patternNames, configFilePath);

        if (boost::filesystem::exists(configFilePath))
        {
            ISM::printGreen("Load objects from config-file.\n");
            loadConstellationFromConfigFile(configFilePath);
        }
        else if (boost::filesystem::exists(mDbFilename))
        {
            ISM::printGreen("Load objects from database.\n");
            if (patternNames.empty())
            {
                std::stringstream ss;
                ss << "Parameter object_configuration_pattern_names must be specified, if database is used!";
                ROS_ERROR_STREAM(ss.str());
                throw std::runtime_error(ss.str());
            }

            tableHandler = ISM::TableHelperPtr(new ISM::TableHelper(mDbFilename));
            if(!tableHandler->recordDataExists())
            {
                std::stringstream ss;
                ss << "The database \"" << mDbFilename << "\" doesn't contain any recordings!\n";
                ROS_ERROR_STREAM(ss.str());
                throw std::runtime_error(ss.str());
            }

            initFromDatabase(patternNames);
        }
        else
        {
            std::stringstream ss;
            ss << "Couldn't load any data! Check path to config-file or to database in the launch-file." << std::endl;
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }

        visualization_pub = nh.advertise<visualization_msgs::MarkerArray>(visualizationTopic, 100);

        ISM::printGreen("WAITING FOR SUBSCRIBER\n");
        while (visualization_pub.getNumSubscribers() == 0) {
            ros::Duration(0.5).sleep();
        }
        visualize();

        ISM::printGreen("Started Object Configuration Generator!\n\n");

        initInteractiveControl();
    }

#include <boost/filesystem/path.hpp>

    ~ObjectConfigurationGenerator()
    {

    }

private:

    /**
     * @brief Read node parameters
     * @param patternName the name of the scene
     */
    void getNodeParameters(std::vector<std::string>& patternNames, std::string& configFilePath)
    {
        if (!nh.getParam("dbfilename", mDbFilename))
        {
            mDbFilename = "";
        }
        ROS_INFO_STREAM("dbfilename: " << mDbFilename);

        nh.getParam("object_configuration_pattern_names", patternNames);
        ROS_INFO_STREAM("object_configuration_pattern_names: ");
        for (unsigned int i = 0; i < patternNames.size(); ++i)
        {
            ROS_INFO_STREAM(patternNames.at(i));
        }

        if (!nh.getParam("baseFrame", baseFrame))
        {
            baseFrame = "";
        }
        ROS_INFO_STREAM("baseFrame: " << baseFrame);

        if (!nh.getParam("visualization_topic", visualizationTopic))
        {
            visualizationTopic = "";
        }
        ROS_INFO_STREAM("visualization_topic: " << visualizationTopic);

        if (!nh.getParam("markerLifetime", markerLifetime))
        {
            markerLifetime = 0;
        }
        ROS_INFO_STREAM("markerLifetime: " << markerLifetime);

        if (!nh.getParam("output_file_path", output_file_path) || output_file_path.empty())
        {
            std::stringstream ss;
            ss << "Parameter output_file_path must be specified!";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
        ROS_INFO_STREAM("output_file_path: " << output_file_path);

        if (!nh.getParam("config_file_path", configFilePath))
        {
            configFilePath = "";
        }
        ROS_INFO_STREAM("config_file_path: " << configFilePath);

    }

    /**
     * @brief Visulize the currently selected pose, all poses that are in the same trajectory and one pose of each other object.
     */
    void visualize()
    {
        for (unsigned int i = 0; i < markerArray.markers.size(); ++i)
        {
            markerArray.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        visualization_pub.publish(markerArray);
        markerArray.markers.clear();

        unsigned int markerCount = 0;

        for (unsigned int i = 0; i < objectAndPoses.size(); ++i)
        {
            std::pair<ISM::ObjectPtr, std::vector<ISM::PosePtr>> pair = objectAndPoses[i];
            std::string path = pair.first->ressourcePath.string();

            bool marked = isMarked(pair.first);

            if (i == objectCounter)
            {
                //visualize selected object
                unsigned int idx = poseCounters[objectCounter];
                markerArray.markers.push_back(VIZ::VizHelperRVIZ::createMeshMarker(pair.second[idx], baseFrame, markerNamespace, markerCount, SELECTED, markerLifetime, path));
                markerCount++;

                ISM::PosePtr pose = pair.second[idx];

                Eigen::Quaternion<double> rot = ISM::GeometryHelper::quatToEigenQuat(pose->quat);
                Eigen::Matrix3d rotation = rot.toRotationMatrix();

                Eigen::Vector3d xAxis = rotation * Eigen::Vector3d::UnitX() * 0.5;
                Eigen::Vector3d yAxis = rotation * Eigen::Vector3d::UnitY() * 0.5;
                Eigen::Vector3d zAxis = rotation * Eigen::Vector3d::UnitZ() * 0.5;

                ISM::PointPtr xEnd = ISM::PointPtr(new ISM::Point(pose->point->eigen.x() + xAxis[0],
                                                   pose->point->eigen.y() + xAxis[1],
                                                   pose->point->eigen.z() + xAxis[2]));
                ISM::PointPtr yEnd = ISM::PointPtr(new ISM::Point(pose->point->eigen.x() + yAxis[0],
                                                   pose->point->eigen.y() + yAxis[1],
                                                   pose->point->eigen.z() + yAxis[2]));
                ISM::PointPtr zEnd = ISM::PointPtr(new ISM::Point(pose->point->eigen.x() + zAxis[0],
                                                   pose->point->eigen.y() + zAxis[1],
                                                   pose->point->eigen.z() + zAxis[2]));
                markerArray.markers.push_back(VIZ::VizHelperRVIZ::createArrowMarkerToPoint(pair.second[idx]->point, xEnd, baseFrame, markerNamespace, markerCount, 0.01f, 0.01f, 0.01f, X_AXIS, markerLifetime));
                markerCount++;
                markerArray.markers.push_back(VIZ::VizHelperRVIZ::createArrowMarkerToPoint(pair.second[idx]->point, yEnd, baseFrame, markerNamespace, markerCount, 0.01f, 0.01f, 0.01f, Y_AXIS, markerLifetime));
                markerCount++;
                markerArray.markers.push_back(VIZ::VizHelperRVIZ::createArrowMarkerToPoint(pair.second[idx]->point, zEnd, baseFrame, markerNamespace, markerCount, 0.01f, 0.01f, 0.01f, Z_AXIS, markerLifetime));
                markerCount++;

                //visualize trajectory of selected object
                for (unsigned int j = 0; j < pair.second.size(); ++j)
                {
                    if (marked && poseCounters[objectCounter] == j)
                    {
                        markerArray.markers.push_back(VIZ::VizHelperRVIZ::createSphereMarker(pair.second[j]->point, baseFrame, markerNamespace, markerCount, sphere_radius, MARKED, markerLifetime));
                    }
                    else
                    {
                        markerArray.markers.push_back(VIZ::VizHelperRVIZ::createSphereMarker(pair.second[j]->point, baseFrame, markerNamespace, markerCount, sphere_radius, IN_CURRENT_TRACK, markerLifetime));
                    }
                    markerCount++;
                }
            }
            else
            {
                //visualize remaining objects
                if (marked)
                {
                    markerArray.markers.push_back(VIZ::VizHelperRVIZ::createMeshMarker(pair.second[poseCounters[i]], baseFrame, markerNamespace, markerCount, MARKED, markerLifetime, path));
                }
                else
                {                    
                    std_msgs::ColorRGBA color = VIZ::VizHelperRVIZ::createColorRGBA(ISM::getColorOfObject(pair.first));
                    markerArray.markers.push_back(VIZ::VizHelperRVIZ::createMeshMarker(pair.second[poseCounters[i]], baseFrame, markerNamespace, markerCount, color, markerLifetime, path));
                }
                markerCount++;
            }
        }

        visualization_pub.publish(markerArray);
    }

    /**
     * @brief If the currently selected pose is marked, it gets unmarked, else it gets marked.
     */
    void markOrUnmark()
    {
        ISM::ObjectPtr obj = objectAndPoses[objectCounter].first;
        if (isMarked(obj))
        {
            ISM::printBlue("\tUNMARKED OBJECT\n");
            markedObjects.erase(obj->type + obj->observedId);
        }
        else
        {
            ISM::printBlue("\tMARKED OBJECT\n");
            markedObjects.insert(obj->type + obj->observedId);
        }
    }

    /**
     * @brief Read tracks from the scenes specified by patternNames from database and store them in the appropriate members.
     * @param patternNames the names of the scenes.
     */
    void initFromDatabase(std::vector<std::string>& patternNames)
    {
        std::vector<ISM::ObjectSetPtr> objectsInPattern;
        for(unsigned int i = 0; i < patternNames.size(); ++i)
        {
            std::string patternName = patternNames.at(i);
            if (tableHandler->getRecordedPatternId(patternName) == 0)
            {
                std::stringstream ss;
                ss << "Could not find pattern " + patternName + " in database " + mDbFilename + "!";
                ROS_ERROR_STREAM(ss.str());
                continue;
            }

            std::vector<ISM::ObjectSetPtr> objects = tableHandler->getRecordedPattern(patternName)->objectSets;
            objectsInPattern.insert(objectsInPattern.end(), objects.begin(), objects.end());
        }
        ISM::TracksPtr tracksInPattern = ISM::TracksPtr(new ISM::Tracks(objectsInPattern));      

        for (size_t it = 0; it < tracksInPattern->tracks.size(); ++it)
        {
            if (it < tracksInPattern->tracks.size() - 1 && !tracksInPattern->tracks[it]->objects.size() == tracksInPattern->tracks[it + 1]->objects.size())
            {
                std::cerr<<"Corrupt database\n";
                exit(-6);
            }
        }

        for (unsigned int i = 0; i < tracksInPattern->tracks.size(); ++i)
        {
            std::vector<ISM::ObjectPtr> objects = tracksInPattern->tracks[i]->objects;
            std::vector<ISM::PosePtr> poses;
            ISM::ObjectPtr obj;

            for (ISM::ObjectPtr object : objects)
            {
                if (object)
                {
                    poses.push_back(object->pose);

                    if(!obj)
                    {
                        obj = ISM::ObjectPtr(new ISM::Object(*object));
                    }
                }
            }

            std::stringstream ss;
            ss << "Loaded " << obj->type << " " << obj->observedId << " from database with " << poses.size() << " poses." << std::endl;
            ISM::printGreen(ss.str());

            objectAndPoses.push_back(std::make_pair(obj, poses));
            poseCounters.push_back(0);
            editFlags.push_back(false);
        }
    }

    enum SelectEntity{Previous = -1, Next = 1};

    /**
     * @brief Switch to the neighbor object.
     */
    void switchObject(SelectEntity neighbor)
    {
        if(objectAndPoses.size() <= 0)
            return;
        objectCounter = ((objectCounter + objectAndPoses.size()) + neighbor) % objectAndPoses.size();


        std::string pose_str = " at ";
        if(editFlags[objectCounter] && (poseCounters[objectCounter] == objectAndPoses[objectCounter].second.size() - 1))
        {
            pose_str += "edited pose";
        }
        else
        {
            pose_str += "pose " + std::to_string(poseCounters[objectCounter]);
        }
        ISM::printBlue("\tNOW AT OBJECT " + objectAndPoses[objectCounter].first->type + " " + objectAndPoses[objectCounter].first->observedId + pose_str + "\n" );
    }

    /**
     * @brief Switch to the neighbor pose in the trajectory.
     */
    void switchPose(SelectEntity neighbor)
    {
        if(objectAndPoses.size() <= 0)
            return;
        poseCounters[objectCounter] = ((poseCounters[objectCounter] + objectAndPoses[objectCounter].second.size()) + neighbor) % objectAndPoses[objectCounter].second.size();
        if (editFlags[objectCounter] && (poseCounters[objectCounter] == objectAndPoses[objectCounter].second.size() - 1))
        {
            ISM::printBlue("\tSWITCHED TO EDITED POSE\n" );
        }
        else
        {
            ISM::printBlue("\tSWITCHED TO POSE " + std::to_string(poseCounters[objectCounter]) + "\n" );
        }
    }

    /**
     * @brief Sets up keyboard control.
     */
    void initInteractiveControl()
    {
        printNormalHelpText();

        //Set-up everything for keyboard input
        tcgetattr( STDIN_FILENO, &originalSettings);           // save original terminal settings
        modifiedSettings = originalSettings;
        modifiedSettings.c_lflag &= ~(ICANON | ECHO);          // disable buffering
        modifiedSettings.c_iflag |= IUCLC;                     // map upper case to lower case
        modifiedSettings.c_cc[VMIN] = 0;                       // minimal characters awaited
        modifiedSettings.c_cc[VTIME] = 0;                      // time keep reading after last byte
        tcsetattr( STDIN_FILENO, TCSANOW, &modifiedSettings);  // apply modified terminal settings
        //Timer to poll for keyboard inputtrack
        keyboardTimer = nh.createTimer(ros::Duration(0.1), &ObjectConfigurationGenerator::keyboardInputCallback, this);
        return;
    }

    /**
     * Print possible commands on console.
     */
    void printNormalHelpText()
    {
        std::stringstream commands;
        commands << "Possible commands:\n"
                 << "\tpress \"w\"\tchoose next object for pose configuration\n"
                 << "\tpress \"s\"\tchoose previous object for pose configuration\n"
                 << "\tpress \"a\"\tchoose previous object pose for pose configuration\n"
                 << "\tpress \"d\"\tchoose next object pose for pose configuration\n"
                 << "\tpress \"m\"\tmark selected pose\n"
                 << "\tpress \"p\"\tshow marked objects\n"
                 << "\tpress \"o\"\tsave marked (red) object poses to file\n"
                 << "\tpress \"e\"\tchange into edit mode\n";

        ISM::printYellow(commands.str() + "\n");
        ISM::printBlue("\tNOW AT OBJECT " + objectAndPoses[objectCounter].first->type + " " + objectAndPoses[objectCounter].first->observedId + "\n" );

        return;
    }

    /**
     * Print edit mode help text.
     */
    void printEditModeHelpText()
    {
        ISM::printRed("");
        ISM::printRed("X-Axis is red\n");
        ISM::printGreen("Y-Axis is green\n");
        ISM::printBlue("Z-Axis is blue\n");

        std::stringstream commands;
        commands << "\nPossible commands:\n"
                 << "\ttype \"s\" and enter\tsave pose changes\n"
                 << "\ttype \"d\" and enter\tdiscard pose changes\n";

        ISM::printYellow(commands.str() + "\n");
    }


    /**
     * Print marked objects on console.
     */
    void printMarkedObjects()
    {
        ISM::printBlue("MARKED OBJECTS:\n");
        for (unsigned int i = 0; i < objectAndPoses.size(); i++)
        {
            if (isMarked(objectAndPoses[i].first))
            {
                std::string pose_str = " at ";
                if(editFlags[i] && (poseCounters[i] == objectAndPoses[i].second.size() - 1))
                {
                    pose_str += "edited pose";
                }
                else
                {
                    pose_str += "pose " + std::to_string(poseCounters[i]);
                }

                ISM::printYellow("\t" + objectAndPoses[i].first->type + " " + objectAndPoses[i].first->observedId + pose_str + "\n");
            }
        }

        return;
    }


    /**
     * @brief Function that handles user keyboard input and calls the appropriate functions
     * @param e not used
     */
    void keyboardInputCallback(const ros::TimerEvent& e)
    {
        int tempKeyValue = getchar();
        tcflush(STDIN_FILENO, TCIFLUSH);  //flush unread input, because we don't want to process old input from buffer.

        // don't accept repeated key input if a key is still pressed.
        if (tempKeyValue < 0)
        {
            keyPressed = false;
            return;
        }
        else if(!keyPressed)
        {
            keyPressed =true;
        }
        else
        {
            return;
        }


        switch(tempKeyValue)
        {
        case KEY_NEXT_OBJECT:
            switchObject(SelectEntity::Next);
            break;
        case KEY_PREV_OBJECT:
            switchObject(SelectEntity::Previous);
            break;
        case KEY_NEXT_POSE:
            switchPose(SelectEntity::Next);
            break;
        case KEY_PREV_POSE:
            switchPose(SelectEntity::Previous);
            break;
        case KEY_SWITCH_EDIT:
            ISM::printBlue("\tSWITCHED INTO EDITING MODE\n");
            keyboardTimer.stop();
            tcsetattr( STDIN_FILENO, TCSANOW, &originalSettings);
            runEditMode();
            break;
        case KEY_MARK:
            markOrUnmark();
            break;        
        case KEY_SAVE:
            writeMarkedPosesToFile();
            ISM::printBlue("\tSAVED OBJECT CONFIGURATION TO " + output_file_path + "\n");
            break;
        case KEY_SHOW:
            printMarkedObjects();
            break;
        }

        visualize();
        return;
    }

    /**
     * @brief Start and run edit mode in which the pose of the selected object can be manipulated.
     */
    void runEditMode()
    {
        std::vector<double> editParameters;
        editParameters.reserve(6);
        unsigned int editCounter = 0;

        ISM::PosePtr copy = ISM::PosePtr(new ISM::Pose(*objectAndPoses[objectCounter].second[poseCounters[objectCounter]]));
        ISM::PosePtr pose = objectAndPoses[objectCounter].second[poseCounters[objectCounter]];

        printEditModeHelpText();

        while (true)
        {
            if (editCounter < 3)
            {
                ISM::printGreen("\tEnter offset along ");
            }
            else
            {
                ISM::printGreen("\tEnter rotation around ");
            }

            if (editCounter % 3 == 0) ISM::printRed("x-axis ");
            if (editCounter % 3 == 1) ISM::printGreen("y-axis ");
            if (editCounter % 3 == 2) ISM::printBlue("z-axis ");

            if (editCounter < 3)
            {
                ISM::printGreen("(in meters): \n\t");
            }
            else
            {
                ISM::printGreen("(in degree): \n\t");
            }

            std::string value;
            std::getline(std::cin, value);

            if (value.compare("s") == 0)
            {
                ISM::printBlue("\tSAVED CHANGES\n\tEXITED EDITING MODE\n");

                objectAndPoses[objectCounter].second[poseCounters[objectCounter]] = copy;
                if (editFlags[objectCounter])
                {
                    objectAndPoses[objectCounter].second.back() = pose;
                }
                else
                {
                    objectAndPoses[objectCounter].second.push_back(pose);
                    editFlags[objectCounter] = true;
                }
                poseCounters[objectCounter] = objectAndPoses[objectCounter].second.size() - 1;

                markedObjects.insert(objectAndPoses[objectCounter].first->type + objectAndPoses[objectCounter].first->observedId);
                ISM::printBlue("\tOBJECT MARKED\n");

                initInteractiveControl();
                break;
            }
            else if (value.compare("d") == 0)
            {
                ISM::printRed("\tDISCARDED CHANGES\n");
                ISM::printBlue("\tEXITED EDITING MODE\n");

                objectAndPoses[objectCounter].second[poseCounters[objectCounter]] = copy;

                initInteractiveControl();
                break;
            }
            else
            {
                try
                {                    
                    double offset = value.empty() ? 0.0 : boost::lexical_cast<double>(value);
                    switch (editCounter)
                    {
                    case 0 :
                        translate(offset, 0, 0, pose);
                        break;
                    case 1 :
                        translate(0, offset, 0, pose);
                        break;
                    case 2 :
                        translate(0, 0, offset, pose);
                        break;
                    case 3 :
                        rotate(offset, 0, 0, pose);
                        break;
                    case 4 :
                        rotate(0, offset, 0, pose);
                        break;
                    case 5 :
                        rotate(0, 0, offset, pose);
                        break;
                    }

                    editCounter = (editCounter + 1) % 6;
                    visualize();
                }
                catch(boost::bad_lexical_cast& e)
                {
                    printEditModeHelpText();
                }

            }
        }
    }


    /**
     * @brief Checks if object is marked.
     */
    bool isMarked(const  ISM::ObjectPtr& obj)
    {
        return markedObjects.count(obj->type + obj->observedId);
    }

    /**
     * @brief Writes the type, observedId, path to the mesh and its pose to an xml file.
     */
    void writeMarkedPosesToFile()
    {
        std::ofstream myfile;
        std::ostringstream s;
        myfile.open (output_file_path);
        myfile << "<Objects>";

        for (unsigned int i = 0; i < objectAndPoses.size(); i++)
        {
            ISM::ObjectPtr obj = objectAndPoses[i].first;
            if (isMarked(obj))
            {
                ISM::PosePtr pose = objectAndPoses[i].second[poseCounters[i]];
                Eigen::Quaterniond poseQuat(pose->quat->eigen.w(),
                            pose->quat->eigen.x(),
                            pose->quat->eigen.y(),
                            pose->quat->eigen.z());
                Eigen::Matrix3d poseMat = poseQuat.toRotationMatrix();
                Eigen::Vector3d poseAngles = poseMat.eulerAngles(0,1,2);

                myfile << "<Object "
                   << "type=\"" << obj->type
                   << "\" id=\"" << obj->observedId
                   << "\" mesh=\"" << obj->ressourcePath.string()
                   << "\" angles=\"euler\">"  << pose->point->eigen.x()
                   << "," << pose->point->eigen.y()
                   << "," << pose->point->eigen.z()
                   << "," << poseAngles[0]*180/M_PI
                   << "," << poseAngles[1]*180/M_PI
                   << "," << poseAngles[2]*180/M_PI
                   << " </Object>";
            }
        }
        myfile << "</Objects>";
        myfile.close();
    }


    /**
     * @brief Rotates the given pose by the values specified by the arguments.
     * @param alpha rotation angle in degree around the x-axis.
     * @param beta rotation angle in degree around the y-axis.
     * @param gamma rotation angle in degree around the z-axis.
     * @param pose the pose that is to be rotated.
     */
    void rotate(double alpha, double beta, double gamma, ISM::PosePtr pose)
    {
        Eigen::Matrix3d rotateBy;
        rotateBy = Eigen::AngleAxisd(alpha * (M_PI / 180), Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(beta * (M_PI / 180), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(gamma * (M_PI / 180), Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> rot = ISM::GeometryHelper::quatToEigenQuat(pose->quat);
        Eigen::Matrix3d original = rot.toRotationMatrix();
        original = original * rotateBy;
        Eigen::Quaternion<double> result(original);
        pose->quat = ISM::GeometryHelper::eigenQuatToQuat(result);
    }

    /**
     * @brief Translates the given pose by the values specified by the arguments
     * @param x the offset in meters by which the pose is translated along the x-axis.
     * @param y the offset in meters by which the pose is translated along the y-axis.
     * @param z the offset in meters by which the pose is translated along the z-axis.
     * @param pose the pose that is to be translated.
     */
    void translate(double x, double y, double z, ISM::PosePtr pose)
    {
        Eigen::Quaternion<double> rot = ISM::GeometryHelper::quatToEigenQuat(pose->quat);
        Eigen::Matrix3d rotationMat = rot.toRotationMatrix();

        Eigen::Vector3d xAxis = rotationMat * Eigen::Vector3d::UnitX() * x;
        Eigen::Vector3d yAxis = rotationMat * Eigen::Vector3d::UnitY() * y;
        Eigen::Vector3d zAxis = rotationMat * Eigen::Vector3d::UnitZ() * z;

        Eigen::Vector3d add = xAxis + yAxis + zAxis;

        pose->point->eigen.x() = pose->point->eigen.x() + add[0];
        pose->point->eigen.y() = pose->point->eigen.y() + add[1];
        pose->point->eigen.z() = pose->point->eigen.z() + add[2];
    }


    /**
     * @brief Loads a configuration from an xml file located at configFilePath;mDbFilename
     * @param configFilePath the location of the config file
     */
    void loadConstellationFromConfigFile(const std::string& configFilePath)
    {
        std::string xml_path = configFilePath;
        ROS_DEBUG_STREAM("Path to objects.xml: " << xml_path);

        try {
            rapidxml::file<> xmlFile(xml_path.c_str());
            rapidxml::xml_document<> doc;
            doc.parse<0>(xmlFile.data());

            rapidxml::xml_node<> *root_node = doc.first_node();
            if (root_node) {
                rapidxml::xml_node<> *child_node = root_node->first_node();
                while (child_node) {
                    rapidxml::xml_attribute<> *type_attribute = child_node->first_attribute("type");
                    rapidxml::xml_attribute<> *id_attribute = child_node->first_attribute("id");
                    rapidxml::xml_attribute<> *mesh_attribute = child_node->first_attribute("mesh");
                    rapidxml::xml_attribute<> *angles_attribute = child_node->first_attribute("angles");

                    if (type_attribute && id_attribute && mesh_attribute && angles_attribute)
                    {
                        std::string angle = angles_attribute->value();
                        std::string pose_string = child_node->value();

                        ISM::PosePtr pose;
                        if (parsePoseString(pose_string, pose, " ,", angle)) {

                            ISM::ObjectPtr obj = ISM::ObjectPtr(new ISM::Object(type_attribute->value(), id_attribute->value(), mesh_attribute->value()));
                            std::vector<ISM::PosePtr> poses;
                            poses.push_back(pose);
                            objectAndPoses.push_back(std::make_pair(obj, poses));
                            markedObjects.insert(obj->type + obj->observedId);
                            poseCounters.push_back(0);
                            editFlags.push_back(false);

                            std::stringstream ss;
                            ss << "Loaded " << obj->type << " " << obj->observedId << " from config-file." << std::endl;
                            ISM::printGreen(ss.str());
                        }
                        else
                        {
                            std::stringstream ss;
                            ss << "Couldn't parse pose for " << type_attribute->value() << " " << id_attribute->value() << " from config-file!" << std::endl;
                            ISM::printRed(ss.str());
                        }
                    }
                    else
                    {
                       ISM::printRed("Couldn't load object from config-file! Check format of the file.\n");
                    }
                    child_node = child_node->next_sibling();
                }
            }
        } catch(std::runtime_error err) {
            ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
        } catch (rapidxml::parse_error err) {
            ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
        }
    }

    /**
     * @brief Creates a ISM::PosePtr from a pose_string
     * @param pose_string contains x, y, z values and angles
     * @param pose the resulting pose is stored here
     * @param delim the symbols that separate values in pose_string
     * @return if the ISM::PosePtr could successfully be created
     */
    bool parsePoseString(std::string pose_string, ISM::PosePtr& pose, std::string delim, std::string angles)
    {
        std::vector<std::string> strvec;

        boost::algorithm::trim_if(pose_string, boost::algorithm::is_any_of(delim));
        boost::algorithm::split(strvec, pose_string, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
        if (strvec.size() == 6 || strvec.size() == 7 ) {
            try {
                    double x = boost::lexical_cast<double>(strvec[0]);
                    double y = boost::lexical_cast<double>(strvec[1]);
                    double z = boost::lexical_cast<double>(strvec[2]);
                    ISM::PointPtr pointPtr = ISM::PointPtr(new ISM::Point(x, y, z));

                    if(angles == "quaternion" && strvec.size() == 7)
                    {
                        double qW = boost::lexical_cast<double>(strvec[3]);
                        double qX = boost::lexical_cast<double>(strvec[4]);
                        double qY = boost::lexical_cast<double>(strvec[5]);
                        double qZ = boost::lexical_cast<double>(strvec[6]);
                        ISM::QuaternionPtr quatPtr = ISM::QuaternionPtr(new ISM::Quaternion(qW, qX, qY, qZ));
                        pose = ISM::PosePtr(new ISM::Pose(pointPtr, quatPtr));
                        return true;

                    }
                    else if(angles == "euler" && strvec.size() == 6)
                    {
                        double qX = boost::lexical_cast<double>(strvec[3]) * (M_PI / 180);
                        double qY = boost::lexical_cast<double>(strvec[4]) * (M_PI / 180);
                        double qZ = boost::lexical_cast<double>(strvec[5]) * (M_PI / 180);
                        Eigen::Matrix3d rotMat;
                        rotMat = Eigen::AngleAxisd(qX, Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(qY,  Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(qZ, Eigen::Vector3d::UnitZ());

                        Eigen::Quaterniond quat(rotMat);
                        ISM::QuaternionPtr quatPtr = ISM::QuaternionPtr(new ISM::Quaternion(quat.w(), quat.x(), quat.y(), quat.z()));
                        pose = ISM::PosePtr(new ISM::Pose(pointPtr, quatPtr));
                        return true;
                    }
                    return false;
            } catch (boost::bad_lexical_cast err) {
                ROS_DEBUG_STREAM("Can't cast node-value. Cast error: " << err.what());
                return false;
            }
        }
        return false;
    }



private:
    ros::NodeHandle nh;
    ISM::TableHelperPtr tableHandler;

    std::vector<std::pair<ISM::ObjectPtr, std::vector<ISM::PosePtr>>> objectAndPoses;
    std::set<std::string> markedObjects;

    std::string mDbFilename;
    std::string baseFrame;
    ros::Publisher visualization_pub;

    std::string visualizationTopic;
    std::string markerNamespace = "object_configuration";

    double markerLifetime;

    unsigned int objectCounter = 0;
    std::vector<unsigned int> poseCounters;
    std::vector<bool> editFlags;

    std::string output_file_path;


    bool keyPressed;

    float sphere_radius = 0.01;

    const std_msgs::ColorRGBA SELECTED = VIZ::VizHelperRVIZ::createColorRGBA(1.0, 0.0, 1.0, 1.0);
    const std_msgs::ColorRGBA MARKED = VIZ::VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 1.0);
    const std_msgs::ColorRGBA IN_CURRENT_TRACK = VIZ::VizHelperRVIZ::createColorRGBA(0, 1.0, 0, 1);

    const std_msgs::ColorRGBA X_AXIS = VIZ::VizHelperRVIZ::createColorRGBA(1.0, 0.0, 0.0, 0.5);
    const std_msgs::ColorRGBA Y_AXIS = VIZ::VizHelperRVIZ::createColorRGBA(0.0, 1.0, 0.0, 0.5);
    const std_msgs::ColorRGBA Z_AXIS = VIZ::VizHelperRVIZ::createColorRGBA(0.0, 0.0, 1.0, 0.5);

    visualization_msgs::MarkerArray markerArray;


    ros::Timer keyboardTimer;

};



int main(int argc, char **argv) {

    //Usual ros node stuff
    ros::init(argc, argv, "object_configuration_generator");
    ObjectConfigurationGenerator* ocg = new ObjectConfigurationGenerator();
    ros::spin();
    delete ocg;

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    tcsetattr( STDIN_FILENO, TCSANOW, &originalSettings);  // restore original terminal settings

}


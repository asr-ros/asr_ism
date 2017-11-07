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
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

//Pkg includes
#include <ros/ros.h>
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <asr_ism_visualizations/vote_visualizer_rviz.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

//ISM includes
#include <ISM/utility/GeometryHelper.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/Util.hpp>



class VoteViewer
{
    public:
        VoteViewer(): nh_("~")
        {
            std::string db_filename;
            std::string base_frame;
            std::string visualization_topic;
            getNodeParameters(db_filename, base_frame, visualization_topic);
            ISM::TableHelperPtr table_helper(new ISM::TableHelper(db_filename));
            if(!table_helper->modelDataExists())
            {
                ISM::printRed("The database \"" + db_filename + "\" doesn't contain a model!\n");
                exit(0);
            }

            init(table_helper);

            visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 10000);
            object_model_visualizer_ = new VIZ::ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);
            vote_visualizer_ = new VIZ::VoteVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);

            timer_ = nh_.createTimer(ros::Duration(1), &VoteViewer::refreshVisualization, this);
        }
  

        void printObjects()
        {
            ISM::printGreen("\n###############################################\n");
            std::cout << scene_name_ << " contains following sub-ISMs and objects (NAME & ID):\n";
            for (std::pair<const std::string, std::map<std::string, std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>>>& object_to_votes : pattern_to_object_to_votes_)
            {
                std::cout  << "\t" << object_to_votes.first << ":\n";
                for (std::pair<const std::string, std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>>& object_name : object_to_votes.second)
                {
                    for (std::pair<const std::string, std::vector<ISM::VoteSpecifierPtr>>& object_id : object_name.second)
                    {
                        std::cout << "\t\t" << object_name.first << "  " << object_id.first << std::endl;
                    }
                }
            }
            ISM::printGreen("###############################################\n");
        }       


        void loadObjectsFromXML() {
            objects_.clear();
            object_to_votes_.clear();

            if(config_file_path_ == "")
            {
                ISM::printRed("Couldn't load object constellation from xml, because the parameter \"config_file_path\" is not specified!\n");
                exit(0);
            }

            std::string xml_path = config_file_path_;
            ROS_DEBUG_STREAM("Path to objects.xml: " << xml_path);

            try {
                rapidxml::file<> xmlFile(xml_path.c_str());
                rapidxml::xml_document<> doc;
                doc.parse<0>(xmlFile.data());

                rapidxml::xml_node<> *root_node = doc.first_node();
                if (root_node)
                {
                    rapidxml::xml_node<> *child_node = root_node->first_node();
                    while (child_node)
                    {
                        rapidxml::xml_attribute<> *type_attribute = child_node->first_attribute("type");
                        rapidxml::xml_attribute<> *id_attribute = child_node->first_attribute("id");
                        rapidxml::xml_attribute<> *mesh_attribute = child_node->first_attribute("mesh");
                        rapidxml::xml_attribute<> *angles_attribute = child_node->first_attribute("angles");
                        if (type_attribute && id_attribute && mesh_attribute && angles_attribute)
                        {
                            std::string type = type_attribute->value();
                            std::string mesh = mesh_attribute->value();
                            std::string id = id_attribute->value();
                            std::string pose_string = child_node->value();
                            std::string angles = angles_attribute->value();
                            ISM::PosePtr pose = ISM::PosePtr(new ISM::Pose());
                            if (parsePoseString(pose_string, pose, " ,", angles))
                            {
                                addObjectAndVotes(ISM::ObjectPtr(new ISM::Object(type, pose, id, mesh)), true);
                            }
                        }
                        child_node = child_node->next_sibling();
                    }
                }
            } catch(std::runtime_error err) {
                ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
            } catch (rapidxml::parse_error err) {
                ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
            }

            if (objects_.empty())
            {
                ISM::printRed("No objects loaded! Check path specified in the launch-file!\n");
                ros::shutdown();
            }
            else
            {
                ISM::printGreen("Objects loaded!\n");
            }
        }



        void generateVisualization()
        {
            object_model_visualizer_->clearAllMarkerOfTopic();
            vote_visualizer_->clearAllMarkerOfTopic();

            object_model_visualizer_->drawObjectModels(objects_);
            vote_visualizer_->addVisualization(object_to_votes_);
        }



        void spinSelectionMode()
        {
            ros::Rate refresh_rate(1); // 1 hz
            while (ros::ok())
            {
                std::string obj_name;
                std::string obj_id;
                ISM::printYellow("\nNEW SELECTION:\n");
                std::cout << "Enter object NAME: ";
                std::getline (std::cin, obj_name);
                std::cout << "Enter object ID: ";
                std::getline (std::cin, obj_id);

                bool exist = this->objectExist(obj_name, obj_id);
                if (exist)
                {
                   ISM::printGreen("SELECTED ");
                   std::cout << obj_name << " with the ID " << obj_id << std::endl;

                   ISM::PosePtr origin(new ISM::Pose(new ISM::Point(0.0, 0.0, 0.0), new ISM::Quaternion(1.0, 0.0, 0.0, 0.0)));
                   std::string resource_path = (type_to_resource_path_.find(obj_name) == type_to_resource_path_.end()) ? "" : type_to_resource_path_[obj_name].string();
                   ISM::ObjectPtr obj(ISM::ObjectPtr(new ISM::Object(obj_name, origin, obj_id, resource_path)));

                   objects_.clear();
                   object_to_votes_.clear();
                   addObjectAndVotes(obj);
                   generateVisualization();
                }
                else
                {
                   ISM::printRed("NO SUCH OBJECT!\n");
                }

                ros::spinOnce();
                refresh_rate.sleep();
            }
        }

    private:

        void getNodeParameters(std::string& db_filename, std::string& base_frame, std::string& visualization_topic)
        {
            if (!nh_.getParam("dbfilename", db_filename))
            {
                db_filename = "";
            }
            ROS_INFO_STREAM("dbfilename: " << db_filename);

            if (!nh_.getParam("visualization_topic", visualization_topic))
            {
                visualization_topic = "";
            }
            ROS_INFO_STREAM("visualization_topic: " << visualization_topic);

            if (!nh_.getParam("baseFrame", base_frame))
            {
                base_frame = "/map";
            }
            ROS_INFO_STREAM("baseFrame: " << base_frame);

            if (!nh_.getParam("sceneName", scene_name_))
            {
                scene_name_ = "";
            }
            ROS_INFO_STREAM("sceneName: " << scene_name_);

            if (!nh_.getParam("config_file_path", config_file_path_))
            {
                config_file_path_ = "";
            }
            ROS_INFO_STREAM("config_file_path: " << config_file_path_);
        }


        void init(ISM::TableHelperPtr table_helper)
        {
            std::vector<std::string> pattern_names = table_helper->getModelPatternNames();
            pattern_to_object_to_votes_ = ISM::PatternToObjectToVoteMap();            
            bool pattern_exist = false;

            for (std::string pattern_name : pattern_names)
            {
                if(pattern_name.find(scene_name_) == 0)
                {
                    std::set<std::pair<std::string, std::string>> objects =  table_helper->getObjectTypesAndIdsBelongingToPattern(pattern_name);
                    pattern_to_object_to_votes_[pattern_name] = table_helper->getVoteSpecifiersForPatternAndObjects(pattern_name, objects);

                    pattern_exist = pattern_exist || !pattern_to_object_to_votes_[pattern_name].empty();
                }
            }            

            if (!pattern_exist)
            {
                ISM::printRed("No pattern in the database, with the name: " + scene_name_ + "\n");
                exit(0);
            }

            type_to_resource_path_ = table_helper->getRessourcePaths();
        }


        bool parsePoseString(std::string pose_in, ISM::PosePtr &pose_out, std::string delim, std::string angles) {
            std::vector<std::string> strvec;

            boost::algorithm::trim_if(pose_in, boost::algorithm::is_any_of(delim));
            boost::algorithm::split(strvec, pose_in, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
            if (strvec.size() == 6 || strvec.size() == 7) {
                try {
                    pose_out->point->eigen.x() = boost::lexical_cast<double>(strvec[0]);
                    pose_out->point->eigen.y() = boost::lexical_cast<double>(strvec[1]);
                    pose_out->point->eigen.z() = boost::lexical_cast<double>(strvec[2]);

                    if(angles == "quaternion" && strvec.size() == 7)
                    {
                        pose_out->quat->eigen.w() = boost::lexical_cast<double>(strvec[3]);
                        pose_out->quat->eigen.x() = boost::lexical_cast<double>(strvec[4]);
                        pose_out->quat->eigen.y() = boost::lexical_cast<double>(strvec[5]);
                        pose_out->quat->eigen.z() = boost::lexical_cast<double>(strvec[6]);
                    }
                    else if(angles == "euler" && strvec.size() == 6)
                    {
                        double euler0,euler1,euler2;
                        euler0 = boost::lexical_cast<double>(strvec[3]);
                        euler1 = boost::lexical_cast<double>(strvec[4]);
                        euler2 = boost::lexical_cast<double>(strvec[5]);

                        Eigen::Matrix3d rotationMatrix;
                        rotationMatrix = Eigen::AngleAxisd(euler0 * (M_PI / 180), Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(euler1 * (M_PI / 180), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(euler2 * (M_PI / 180), Eigen::Vector3d::UnitZ());

                        Eigen::Quaternion<double> result(rotationMatrix);
                        pose_out->quat->eigen.w() = result.w();
                        pose_out->quat->eigen.x() = result.x();
                        pose_out->quat->eigen.y() = result.y();
                        pose_out->quat->eigen.z() = result.z();
                    }
                    else
                    {
                        ROS_ERROR("Invalid XML syntax.");
                        nh_.shutdown();
                    }

                    return true;
                } catch (boost::bad_lexical_cast err) {
                    ROS_DEBUG_STREAM("Can't cast node-value. Cast error: " << err.what());
                }
            }
            return false;
        }


        void addObjectAndVotes(ISM::ObjectPtr object, bool recursive = false)
        {
            ROS_DEBUG_STREAM("Add Object " << object->type << "  with id " << object->observedId << ". [addObjectAndVotes(..)]");
            objects_.push_back(object);

            for (ISM::PatternToObjectToVoteMap::iterator pat_it = pattern_to_object_to_votes_.begin(); pat_it != pattern_to_object_to_votes_.end(); ++pat_it)
            {
                ISM::ObjectToVoteMap::iterator obj_name_it = pat_it->second.find(object->type);
                if (obj_name_it != pat_it->second.end())
                {
                    std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>::iterator obj_id_it = obj_name_it->second.find(object->observedId);
                    if (obj_id_it != obj_name_it->second.end())
                    {
                        if(obj_id_it->second.size() > 0)
                        {
                            object_to_votes_[object].insert(object_to_votes_[object].begin(), obj_id_it->second.begin(), obj_id_it->second.end());

                            // if recursive adding is true and the object has a selfvote so add sub_ism object with same pose.
                            if(recursive && pat_it->first.find("sub") != std::string::npos && ISM::GeometryHelper::isSelfVote(obj_id_it->second[0]))
                            {
                                ROS_DEBUG_STREAM("Object " << object->type << "  with id " << object->observedId << " is the reference object of sub-ism " << pat_it->first << ". [addObjectAndVotes(..)]");
                                ISM::ObjectPtr obj(ISM::ObjectPtr(new ISM::Object(pat_it->first, object->pose, "")));
                                addObjectAndVotes(obj, true);
                            }
                        }
                    }
                }
            }

            if (object_to_votes_.find(object) != object_to_votes_.end())
            {
                ROS_DEBUG_STREAM("For Object " << object->type << "  with id " << object->observedId << " are " <<  object_to_votes_[object].size()<< " votes in the model for " << scene_name_ << ". [addObjectAndVotes(..)]");
            }
            else
            {
                ROS_DEBUG_STREAM("No votes for Object " << object->type << "  with id " << object->observedId << " in the model for " << scene_name_ << ". [addObjectAndVotes(..)]");
            }
        }


        bool objectExist(std::string& obj_name, std::string& obj_id)
        {
            for (ISM::PatternToObjectToVoteMap::iterator pat_it = pattern_to_object_to_votes_.begin(); pat_it != pattern_to_object_to_votes_.end(); ++pat_it)
            {
                ISM::ObjectToVoteMap::iterator obj_name_it = pat_it->second.find(obj_name);
                if (obj_name_it != pat_it->second.end())
                {
                    std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>::iterator obj_id_it = obj_name_it->second.find(obj_id);
                    if (obj_id_it != obj_name_it->second.end())
                    {
                        return true;
                    }
                }
            }

            return false;
        }


        void refreshVisualization(const ros::TimerEvent& e)
        {
            unsigned int previous_number = number_of_subscriber_;
            number_of_subscriber_ = visualization_publisher_.getNumSubscribers();
            if(number_of_subscriber_ > 0 && (number_of_subscriber_ > previous_number))
            {
                ISM::printGreen("Publish vote marker!\n");
                vote_visualizer_->publishCollectedMarkers();
                object_model_visualizer_->publishCollectedMarkers();
                std::cout<<"\n";
            }
        }


    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;

        std::string scene_name_;
        ISM::PatternToObjectToVoteMap pattern_to_object_to_votes_;
        std::vector<ISM::ObjectPtr> objects_;
        std::map<ISM::ObjectPtr, std::vector<ISM::VoteSpecifierPtr>> object_to_votes_;
        std::map<std::string, boost::filesystem::path> type_to_resource_path_;

        std::string config_file_path_;

        //Visualization stuff
        ros::Publisher visualization_publisher_;
        VIZ::ObjectModelVisualizerRVIZ* object_model_visualizer_;
        VIZ::VoteVisualizerRVIZ* vote_visualizer_;
        unsigned int number_of_subscriber_;

};

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "modelViewer");
    VoteViewer* vote_viewer = new VoteViewer();

    vote_viewer->printObjects();

    while (true)
    {
        std::string load_config_query;
        ISM::printYellow("\nDo you want to use object constellation from an xml-file?\n");
        std::cout << "[y]es or [n]o: ";
        std::getline (std::cin, load_config_query);
        if (load_config_query == "y")
        {
            vote_viewer->loadObjectsFromXML();
            vote_viewer->generateVisualization();
            ros::spin(); // spin so visualization will be refreshed if needed
            break;
        }
        else if (load_config_query == "n")
        {
            vote_viewer->spinSelectionMode();
            break;
        }
        else
        {
            std::cout << load_config_query << " is not valid, try again!" << std::endl;
        }
    }


    delete vote_viewer;
    return 0;
}

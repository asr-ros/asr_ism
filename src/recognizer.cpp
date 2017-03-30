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
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

//Pkg includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include <asr_msgs/AsrObject.h>

#include <asr_ism_visualizations/ism_result_visualizer_rviz.hpp>
#include <asr_ism_visualizations/ism_voting_visualizer_rviz.hpp>
#include <asr_ism_visualizations/ism_pose_prediction_visualizer_rviz.hpp>
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <asr_ism_visualizations/VizHelperRVIZ.hpp>

#include <pose_prediction_ism/pose_predictor.h>
#include <pose_prediction_ism/shortest_path.h>

//ISM includes
#include <ISM/recognizer/Recognizer.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/Util.hpp>
#include <ISM/common_type/Pose.hpp>
#include <ISM/typedef.hpp>

#include "ISM/rating/BaseRater.hpp"
#include "ISM/rating/SimpleRater.hpp"
#include "ISM/rating/APORater.hpp"
#define SIMPLE_RATER 0
#define APO_RATER 1

//Local includes
#include <asr_ism/recognizerConfig.h>
#include "asr_ism/scene_configurator.hpp"


/**********************/
/*** Keyboard input ***/
#define KEYCODE_BUFFERVOTES 0x23//#
#define KEYCODE_VISUALIZE 0x76  //v

#define KEYCODE_BUFFERSCENE 0x2B //+
#define KEYCODE_PREDICTSCENE 0x70 //p
#define KEYCODE_NEXTREF 0x79    //y
#define KEYCODE_PREVREF 0x3C    //<
#define KEYCODE_NEXTOBJECT 0x63 //c
#define KEYCODE_PREVOBJECT 0x78 //x
#define KEYCODE_NEXTPOSE 0x6E   //n
#define KEYCODE_PREVPOSE 0x62   //b

#define KEYCODE_HELP 0x68       //h

#define KEYCODE_NEXT_RESULT 0x71           //q
#define KEYCODE_PREV_RESULT 0x77           //w
#define KEYCODE_VIZ_SELECTED_RESULT 0x65   //e
#define KEYCODE_VIZ_ALL_RESULTS 0x72       //r
/**********************/

/**
 * Recognizer class. ROS Node wrapper for Recognizer from libism.
 *
 * @author Reno Reckling, Pascal Meissner
 * @version See SVN
 */
class Recognizer 
{
    public: 


        /**
         * Constructor processing parameters of scene recognition and setting up ros node.
         */
        Recognizer() : nh_("~")
        {
        
            std::string visualization_topic;
            //Extract cli parameters of ros node for being used in this program.
            getNodeParameters(visualization_topic);

            if (db_filename_ != "")
            {
                ism_recognizer_ = ISM::RecognizerPtr(new ISM::Recognizer(db_filename_, bin_size_, max_projection_angle_deviation_, rater_type_));
            }
            else
            {
                throw std::runtime_error("No db specified");
            }


            object_set_buffer_ = ISM::ObjectSetPtr(new ISM::ObjectSet());           

            // init viz
            visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 100);
            recogition_result_visualizer_ = VIZ::ISMResultVisualizerRVIZPtr(new VIZ::ISMResultVisualizerRVIZ(visualization_publisher_, ros::NodeHandle(nh_.getNamespace() + "/result_visualizer/")));
            voting_space_visualizer_ = VIZ::ISMVotingVisualizerRVIZPtr(new VIZ::ISMVotingVisualizerRVIZ(visualization_publisher_, bin_size_, ros::NodeHandle(nh_.getNamespace() + "/voting_visualizer/")));
            object_model_visualizer_ = new VIZ::ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame_, "", 0);

            // init pose prediction stuff
            ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(db_filename_));
            object_type_to_ressource_path_map_ = table_helper->getRessourcePaths();

            pose_predictor_ = new pose_prediction_ism::ShortestPath(db_filename_);
            ism_pose_prediction_visualizer_ = VIZ::ISMPosePredictionVisualizerRVIZPtr(new VIZ::ISMPosePredictionVisualizerRVIZ(visualization_publisher_, bin_size_, max_projection_angle_deviation_, object_type_to_ressource_path_map_,
                                                                                      ros::NodeHandle(nh_.getNamespace() + "/pose_prediction_visualizer/"), ros::NodeHandle(nh_.getNamespace() + "/valid_position_visualizer/")));

            // set-up dynamic reconfigure
            dynamic_reconfigure::Server<asr_ism::recognizerConfig>::CallbackType reconf_callback = boost::bind(&Recognizer::dynamicReconfCallback, this, _1, _2);
            dyn_reconf_server_.setCallback(reconf_callback);

            // init scene configurator
            scene_configurator_ = new SceneConfigurator(boost::bind(&Recognizer::recognizeScene, this, _1),
                                                        boost::bind(&Recognizer::processKeyboardInput, this, _1),
                                                        boost::bind(&Recognizer::processObjectInput, this, _1));

            printAdditionalHelpText();
        }

         ~Recognizer()
        {
            delete scene_configurator_;
        }


        /**
        * Recognize object configuration.
        *
        * @param object_set Object configuration which should be recognized.
        */
        void recognizeScene(ISM::ObjectSetPtr object_set)
        {
            ISM::printYellow("Recognize scene for object set:\n");
            storeObjectsToXML(object_set);

            if(buffer_next_scene_)
            {
                object_set_buffer_->objects.clear();
                object_set_buffer_->objects.insert(object_set_buffer_->objects.begin(), object_set->objects.begin(), object_set->objects.end());
                buffer_next_scene_ = false;
            }



            ROS_INFO_STREAM("-----------------------------------------------------------");
            ROS_INFO_STREAM("Calculating scene recognition results. Please wait.");

            struct timeval start, end;
            //Reset start
            gettimeofday(&start, NULL);

            //Here actual scene recognition takes place.
            std::vector<ISM::RecognitionResultPtr> all_results = ism_recognizer_->recognizePattern(object_set, 0, max_number_of_results_per_pattern_);

            gettimeofday(&end, NULL);

            long recTime, seconds, useconds;
            seconds = end.tv_sec - start.tv_sec;
            useconds = end.tv_usec - start.tv_usec;
            recTime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
            ROS_INFO_STREAM("elapsed time: " << recTime << "ms");

            results_buffer_ = all_results;
            ROS_INFO_STREAM("results: (" << results_buffer_.size() << ")");

            for (unsigned int idx = 0; idx < results_buffer_.size(); idx++)
            {
                ROS_INFO_STREAM("********************************************");
                ROS_INFO_STREAM("Result Index: " << idx);
                ROS_INFO_STREAM("Number of Combinations: " << results_buffer_[idx]->getNumberOfCombinations());
                ROS_INFO_STREAM(results_buffer_[idx]);
                ISM::printBlue("********** Vote and Score Details ************\n");
                printAdditionalResultInformation(results_buffer_[idx]);
                ISM::printBlue("**********************************************\n\n");
            }

            visualizeAllResults();

            selected_result_index_ = 0;
            ROS_INFO_STREAM("-----------------------------------------------------------");
        }


        /**
         * Process input from keyboard.
         *
         * @param key ASCII of keyboard key.
         */
        void processKeyboardInput(int key)
        {
            switch(key)
            {
                case KEYCODE_HELP:
                    printAdditionalHelpText();
                    break;
                case KEYCODE_BUFFERVOTES:
                    ISM::printBlue("\tBUFFER VOTES\n");
                    ism_recognizer_->setVotingSpaceToBeBuffered(voting_space_visualizer_->getPatternName());
                    break;
                case KEYCODE_VISUALIZE:
                    ISM::printBlue("\tDRAW VOTES\n");
                    clearAllMarker();
                    voting_space_visualizer_->addVisualization(ism_recognizer_->getBufferedVotingSpace(), results_buffer_);
                    break;
                case KEYCODE_BUFFERSCENE:
                    ISM::printBlue("\tBUFFER SCENE\n");
                    buffer_next_scene_ = true;
                    break;
                case KEYCODE_PREDICTSCENE:
                    predictScene();
                    break;
                case KEYCODE_NEXTREF:
                    ISM::printBlue("\tNEXT REFERENCE\n");
                    if(results_buffer_.size() > 0)
                        scene_counter_ = (scene_counter_ + 1) % results_buffer_.size();
                    break;
                case KEYCODE_PREVREF:
                    ISM::printBlue("\tPREVIOUS REFERENCE\n");
                    if(results_buffer_.size() > 0)
                        scene_counter_ = (scene_counter_ - 1 + results_buffer_.size()) % results_buffer_.size();
                    break;
                case KEYCODE_NEXTOBJECT:
                    ISM::printBlue("\tNEXT OBJECT\n");
                    ism_pose_prediction_visualizer_->nextObject();
                    break;
                case KEYCODE_PREVOBJECT:
                    ISM::printBlue("\tPREVIOUS OBJECT\n");
                    ism_pose_prediction_visualizer_->prevObject();
                    break;
                case KEYCODE_NEXTPOSE:
                    ISM::printBlue("\tNEXT POSE\n");
                    ism_pose_prediction_visualizer_->nextPose();
                    break;
                case KEYCODE_PREVPOSE:
                    ISM::printBlue("\tPREVIOUS POSE\n");
                    ism_pose_prediction_visualizer_->prevPose();
                    break;                
                case KEYCODE_NEXT_RESULT:
                    if(results_buffer_.size() > 0)
                    {
                        ISM::printBlue("\tRESULT: ");
                        selected_result_index_ = ((unsigned int)selected_result_index_ + 1 < results_buffer_.size()) ? selected_result_index_ + 1 : 0;
                        std::cout << selected_result_index_ << std::endl;
                        visualizeSelectedResult();
                    }
                    break;
                case KEYCODE_PREV_RESULT:
                    if(results_buffer_.size() > 0)
                    {
                        ISM::printBlue("\tRESULT: ");
                        selected_result_index_ = (selected_result_index_ - 1 >= 0) ? selected_result_index_ - 1 : results_buffer_.size() - 1;
                        std::cout << selected_result_index_ << std::endl;
                        visualizeSelectedResult();
                    }
                    break;
                case KEYCODE_VIZ_SELECTED_RESULT:
                    if(results_buffer_.size() > 0)
                    {
                        ISM::printBlue("\tVISUALIZE RESULT: ");
                        std::cout << selected_result_index_ << std::endl;
                        visualizeSelectedResult();
                    }
                    break;
                case KEYCODE_VIZ_ALL_RESULTS:
                    if(results_buffer_.size() > 0)
                    {
                        ISM::printBlue("\tVISUALIZE ALL RESULTS\n");
                        visualizeAllResults();
                    }
                    break;
            }

            return;
        }

        /**
         * Further processing of incoming objects.
         *
         * @param object Incoming object.
         */
        void processObjectInput(ISM::ObjectPtr object)
        {
            ism_pose_prediction_visualizer_->calculateDistColor(object);
            return;
        }

    private:

        ros::NodeHandle nh_;
        dynamic_reconfigure::Server<asr_ism::recognizerConfig> dyn_reconf_server_;
        SceneConfigurator* scene_configurator_;

        //Core functionality
        ISM::RecognizerPtr ism_recognizer_;
        double bin_size_;
        double max_projection_angle_deviation_;
        std::string db_filename_;
        int rater_type_;
        int max_number_of_results_per_pattern_;

        // store to XML functionality
        bool enable_storing_config_to_xml_;
        std::string configuration_folder_path_;
        unsigned int configuration_file_counter_ = 1;

        //Visualization stuff
        ros::Publisher visualization_publisher_;
        std::string base_frame_;
        int selected_result_index_ = 0;

        VIZ::ISMResultVisualizerRVIZPtr recogition_result_visualizer_;
        VIZ::ISMVotingVisualizerRVIZPtr voting_space_visualizer_;
        VIZ::ISMPosePredictionVisualizerRVIZPtr ism_pose_prediction_visualizer_;
        VIZ::ObjectModelVisualizerRVIZ* object_model_visualizer_;

        //Buffers last results for ism visualization
        std::vector<ISM::RecognitionResultPtr> results_buffer_;

        //Buffers last objectset for pose prediction visualization
        ISM::ObjectSetPtr object_set_buffer_;
        bool buffer_next_scene_ = false;

        //PosePrediction stuff
        pose_prediction_ism::PosePredictor* pose_predictor_;
        std::map<std::string, boost::filesystem::path> object_type_to_ressource_path_map_;
        int scene_counter_ = 0;
        bool valid_position_vis_ = false;
        double pose_prediction_sampel_faktor_ = 0.5;
        size_t number_of_objects_ = 0;


        /**
         * Print additional informations from recognition results.
         */
        void storeObjectsToXML(ISM::ObjectSetPtr object_set)
        {
            //Don`t store the configuration while in active recognition or while storing disabled.
            if (scene_configurator_->getAutomaticProcessingActive() || !enable_storing_config_to_xml_)
                return;

            //Don`t do anything if no path was provided that specifies where the configuration should be stored.
            if (configuration_folder_path_.empty())
                return;


            if (object_set->objects.size() > 0)
            {
                std::ofstream myfile;
                std::string filePath = configuration_folder_path_ + "/" +  "configuration_" + std::to_string(configuration_file_counter_++) + ".xml";
                myfile.open(filePath);
                myfile << "<Objects>";

                for (ISM::ObjectPtr object : object_set->objects)
                {
                    ISM::PosePtr pose = object->pose;
                    myfile << "<Object "
                               << "type=\"" << object->type
                               << "\" id=\"" << object->observedId
                               << "\" mesh=\"" << object->ressourcePath.string()
                               << "\" angles=\"quaternion\">"
                               << pose->point->eigen.x()
                               << "," << pose->point->eigen.y()
                               << "," << pose->point->eigen.z()
                               << "," << pose->quat->eigen.w()
                               << "," << pose->quat->eigen.x()
                               << "," << pose->quat->eigen.y()
                               << "," << pose->quat->eigen.z()
                           << " </Object>";
                }

                myfile << "</Objects>";
                myfile.close();
            }
        }


        /**
        * Print additional informations from recognition results.
        */
        void printAdditionalResultInformation(ISM::RecognitionResultPtr& result)
        {
            ISM::RaterPtr rater;
            switch (rater_type_)
            {
                case APO_RATER:
                    rater = ISM::RaterPtr(new ISM::APORater(bin_size_, max_projection_angle_deviation_));
                    break;
                case SIMPLE_RATER:
                    rater = ISM::RaterPtr(new ISM::SimpleRater(bin_size_, max_projection_angle_deviation_));
                    break;
            }

            std::stringstream sceneInfo;
            sceneInfo << "SCENE: " << result->patternName << "     score: " << result->confidence << "     # of combinations: " << result->getNumberOfCombinations()<< "\n";
            ISM::printGreen(sceneInfo.str());

            double weight = 0.0;
            for(ISM::ObjectPtr o : result->recognizedSet->objects)
            {
                weight += o->weight;
            }
            std::cout << "weight=" << weight << " expectedWeight=" << result->summarizedVotes.front().first->weightDenominator << "\n";

            ISM::printYellow("\n####### VOTE INFORMATIONS #######\n");
            for (ISM::SummarizedVotedPosePtr& summarizedVote : result->summarizedVotes)
            {
                std::stringstream voteInfo;
                voteInfo << summarizedVote.first->source->type << " " << summarizedVote.first->source->observedId << "   # of summarized votes: " << summarizedVote.second.first << "\n";
                ISM::printGreen(voteInfo.str());
                rater->printRatingAtBackProjectionLevel(summarizedVote.first, result->referencePose, nullptr);
            }
            ISM::printYellow("#################################\n");

            if (result->subPatterns.empty())
            {
                return;
            }

            ISM::printYellow("\n##### SUBSCENE INFORMATIONS #####\n");
            for (ISM::RecognitionResultPtr sub : result->subPatterns)
            {
                ISM::printBlue(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
                ISM::printYellow("SUB");
                printAdditionalResultInformation(sub);
                ISM::printBlue("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
            }
            ISM::printYellow("#################################\n");

            return;
        }


        /**
         * Extract an vector of all real objects (objects which don’t represent ISMs) from result.
         *
         * @return Extracted objects.
         */
        std::vector<ISM::ObjectPtr> extractRealObjects(const ISM::RecognitionResultPtr result)
        {
            std::vector<ISM::ObjectPtr> ret_objects;

            for(ISM::ObjectPtr obj : result->recognizedSet->objects )
            {
                if( obj->type.find( result->patternName ) == std::string::npos )
                {
                   ret_objects.push_back(obj);
                }
            }

            // extract objects from sub-isms
            std::vector<ISM::ObjectPtr> temp_objects;
            for (const ISM::RecognitionResultPtr sub_result : result->subPatterns )
            {
                temp_objects = extractRealObjects(sub_result);
                ret_objects.insert(ret_objects.end(), temp_objects.begin(), temp_objects.end());
            }

            return ret_objects;
        }


        /**
         * Print additional commands on console.
         */
        void printAdditionalHelpText()
        {
            std::stringstream commands;

            commands << "Additional recognizer commands:\n"
                     << "\tpress \"#\"\t-buffer next recognition for visualization\n"
                     << "\tpress \"v\"\t-show visualization of votingspace\n"
                     << "\n"
                     << "\tpress \"+\"\t-buffer next scene for pose prediction\n"
                     << "\tpress \"p\"\t-show visualization of pose prediction\n"
                     << "\tpress \"<\"\t-choose previous reference for pose prediction\n"
                     << "\tpress \"y\"\t-choose next reference for pose prediction\n"
                     << "\tpress \"x\"\t-choose previous object for pose prediction\n"
                     << "\tpress \"c\"\t-choose next object for pose prediction\n"
                     << "\tpress \"b\"\t-choose previous object pose for pose prediction\n"
                     << "\tpress \"n\"\t-choose next object pose for pose prediction\n"
                     << "\n"
                     << "\tpress \"q\"\t-select previous result from all results and visualize it\n"
                     << "\tpress \"w\"\t-select next result from all results and visualize it\n"
                     << "\tpress \"e\"\t-visualize selected result\n"
                     << "\tpress \"r\"\t-visualize all results\n";


            ISM::printYellow(commands.str() + "\n");

            return;
        }


        /**
         * Visualize current selected result from last recognition.
         */
        void visualizeSelectedResult()
        {
            clearAllMarker();
            recogition_result_visualizer_->setSceneCount(results_buffer_.size()); //TODO: should namespace really be incremented in viz-code?
            object_model_visualizer_->drawObjectModels(extractRealObjects(results_buffer_[selected_result_index_]));
            recogition_result_visualizer_->addVisualization(results_buffer_[selected_result_index_]);
        }


        /**
         * Visualize all results from last recognition.
         */
        void visualizeAllResults()
        {
            clearAllMarker();
            recogition_result_visualizer_->setSceneCount(results_buffer_.size());

            for (const ISM::RecognitionResultPtr result : results_buffer_)
            {
                object_model_visualizer_->drawObjectModels(extractRealObjects(result));
                recogition_result_visualizer_->addVisualization(result);
            }
        }


        /**
         * Deletes all marker in RVIZ.
         *
         * @brief clearAllMarker
         */
        void clearAllMarker()
        {
            ism_pose_prediction_visualizer_->clearAllMarkerOfTopic();
            voting_space_visualizer_->clearAllMarkerOfTopic();
            recogition_result_visualizer_->clearAllMarkerOfTopic();
        }


        void predictScene()
        {
            ISM::printBlue("\tPREDICT SCENE\n");
            clearAllMarker();
            if(valid_position_vis_)
            {
                pose_predictor_->enableRandom(std::sqrt(1.0 / 3.0) * bin_size_ * 0.5, max_projection_angle_deviation_);
            }
            if(object_set_buffer_->objects.size() == 0)
            {
                ROS_ERROR("No Objects in scenebuffer, skipping prediction visualization.");
                return;
            }

            ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(db_filename_));
            std::set<std::string> all_objects_in_pattern = table_helper->getObjectsInPattern(results_buffer_[scene_counter_]->patternName);

            pose_prediction_ism::FoundObjects fos;
            for(std::size_t i = 0; i < object_set_buffer_->objects.size(); i++)
            {
                asr_msgs::AsrObject asr_o;
                asr_o.type = object_set_buffer_->objects[i]->type;
                asr_o.identifier = object_set_buffer_->objects[i]->observedId;
                all_objects_in_pattern.erase(object_set_buffer_->objects[i]->type + object_set_buffer_->objects[i]->observedId);
                fos.push_back(asr_o);
            }

            ROS_INFO_STREAM(results_buffer_[scene_counter_]->patternName);
            if(all_objects_in_pattern.size() == 0)
            {
                ROS_INFO("Scene complete, nothing to predict.");
                return;
            }
            pose_predictor_->setFoundObjects(fos);
            ISM::PosePtr referencePosePtr = results_buffer_[scene_counter_]->referencePose;
            pose_predictor_->predictUnfoundPoses(referencePosePtr, results_buffer_[scene_counter_]->patternName, pose_prediction_sampel_faktor_);
            pose_prediction_ism::AttributedPointCloud current_point_cloud = pose_predictor_->getAttributedPointCloud();
            ism_pose_prediction_visualizer_->addVisualization(results_buffer_[scene_counter_], current_point_cloud, valid_position_vis_);
            if(valid_position_vis_)
            {
                pose_predictor_->disableRandom();
            }

            return;
        }


        void dynamicReconfCallback(asr_ism::recognizerConfig &config, uint32_t level)
        {
            ROS_DEBUG_STREAM("Parameters updated.");

            voting_space_visualizer_->releaseCallback();
            ism_pose_prediction_visualizer_->releaseCallback();

            voting_space_visualizer_ = VIZ::ISMVotingVisualizerRVIZPtr(new VIZ::ISMVotingVisualizerRVIZ(visualization_publisher_, bin_size_, ros::NodeHandle(nh_.getNamespace() + "/voting_visualizer/")));
            ism_pose_prediction_visualizer_ = VIZ::ISMPosePredictionVisualizerRVIZPtr(new VIZ::ISMPosePredictionVisualizerRVIZ(visualization_publisher_, bin_size_, max_projection_angle_deviation_, object_type_to_ressource_path_map_,
                                                                                    ros::NodeHandle(nh_.getNamespace() + "/pose_prediction_visualizer/"), ros::NodeHandle(nh_.getNamespace() + "/valid_position_visualizer/")));

            object_model_visualizer_ = new VIZ::ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame_, "", config.capture_interval);

            pose_prediction_sampel_faktor_ = config.posePredictionSampelFaktor;
            valid_position_vis_ = config.validPositionVis;
        }


        /**
        * Extracts parameters of already launched ros node and prints them on cli.
        *
        * @param pVisualizationTopic ROS topic on which visualizations detected scenes are published.
        */
        void getNodeParameters(std::string& visualization_topic)
        {         
            if (!nh_.getParam("dbfilename", db_filename_))
            {
                db_filename_ = "";
            }
            ROS_INFO_STREAM("dbfilename: " << db_filename_);

            if (!nh_.getParam("baseFrame", base_frame_))
            {
                base_frame_ = "/map";
            }
            ROS_INFO_STREAM("baseFrame: " << base_frame_);

            if (!nh_.getParam("bin_size", bin_size_))
            {
                bin_size_ = 0.1;
            }
            ROS_INFO_STREAM("bin_size: " << bin_size_);

            if (!nh_.getParam("maxProjectionAngleDeviation", max_projection_angle_deviation_))
            {
                max_projection_angle_deviation_ = 30;
            }
            ROS_INFO_STREAM("maxProjectionAngleDeviation: " << max_projection_angle_deviation_);

            if (!nh_.getParam("maxNumberOfResultsPerPattern", max_number_of_results_per_pattern_))
            {
                max_number_of_results_per_pattern_ = 1;
            }
            ROS_INFO_STREAM("maxNumberOfResultsPerPattern: " << max_number_of_results_per_pattern_);

            if (!nh_.getParam("raterType", rater_type_))
            {
            rater_type_ = 0;
            }
            ROS_INFO_STREAM("raterType: " << rater_type_);

            if (!nh_.getParam("enableStoringConfigToXml", enable_storing_config_to_xml_))
            {
                enable_storing_config_to_xml_ = false;
            }
            ROS_INFO_STREAM("enableStoringConfigToXml: " << enable_storing_config_to_xml_);

            if (!nh_.getParam("configurationFolderPath", configuration_folder_path_))
            {
                configuration_folder_path_ = "";
            }
            ROS_INFO_STREAM("configurationFolderPath: " << configuration_folder_path_);

            if (!nh_.getParam("visualization_topic", visualization_topic))
            {
                visualization_topic = "";
            }
            ROS_INFO_STREAM("visualization_topic: " << visualization_topic);
        }
};



int main(int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "recognizer");
    Recognizer* recognizer = new Recognizer();

    ros::spin();

    //clean after shutdown
    delete recognizer;


    return 0;
}

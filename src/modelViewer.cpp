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
#include <boost/filesystem.hpp>

//Pkg includes
#include <ros/ros.h>

#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <asr_ism_visualizations/model_visualizer_rviz.hpp>

//ISM includes
#include <ISM/common_type/Tracks.hpp>
#include <ISM/utility/GeometryHelper.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/Util.hpp>


class ModelViewer
{
    public:
        ModelViewer(): nh_("~")
        {
            std::string db_filename;
            std::string base_frame;
            std::string visualization_topic;

            getNodeParameters(db_filename, base_frame, visualization_topic);
            table_helper_ = ISM::TableHelperPtr(new ISM::TableHelper(db_filename));
            if(!table_helper_->recordDataExists())
            {
                ISM::printRed("The database \"" + db_filename + "\" doesn't contain any recordings!\n");
                exit(0);
            }
            if(!table_helper_->modelDataExists())
            {
                ISM::printRed("The database \"" + db_filename + "\" doesn't contain a model!\n");
                exit(0);
            }

            initObjectTracks();
            initVotes();
            initISMObjectTrack();


            visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 10000);
            model_visualizer_ = new VIZ::ModelVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);

            ISM::printGreen("Generate and publish models for " + scene_name_ + "!\n");
            ISM::printGreen("\t(sub)pattern:\n");
            number_of_subscriber_ = visualization_publisher_.getNumSubscribers();
            for (std::pair<const std::string, std::map<int, std::vector<ISM::VoteSpecifierPtr>>>& track_index_to_votes : pattern_to_track_index_to_votes_)
            {
                ISM::printBlue("\t\t" + track_index_to_votes.first + "\n");
                model_visualizer_->addVisualization(track_index_to_votes.first, tracks_, ism_object_tracks_, track_index_to_votes.second);
            }

            timer_ = nh_.createTimer(ros::Duration(1), &ModelViewer::refreshVisualization, this);
        }
  
    private:

        void getNodeParameters(std::string& db_filename, std::string& base_frame, std::string& visualization_topic)
        {
            if (!nh_.getParam("dbfilename", db_filename))
            {
                db_filename = "";
            }
            ROS_INFO_STREAM("dbfilename: " << db_filename);

            if (!nh_.getParam("baseFrame", base_frame))
            {
                base_frame = "/map";
            }
            ROS_INFO_STREAM("baseFrame: " << base_frame);

            if (!nh_.getParam("visualization_topic", visualization_topic))
            {
                visualization_topic = "";
            }
            ROS_INFO_STREAM("visualization_topic: " << visualization_topic);

            if (!nh_.getParam("sceneName", scene_name_))
            {
                scene_name_ = "";
            }
            ROS_INFO_STREAM("sceneName: " << scene_name_);
        }


        void initObjectTracks()
        {
            ISM::RecordedPatternPtr recorded_pattern = table_helper_->getRecordedPattern(scene_name_);
            if (recorded_pattern == nullptr)
            {
                ISM::printRed("No recorded pattern in the database, with the name: " + scene_name_ + "\n");
                exit(0);
            }

            std::vector<ISM::ObjectSetPtr> recorded_object_sets = recorded_pattern->objectSets;
            if (recorded_object_sets.empty())
            {
                ISM::printRed("No tracks in the database for the pattern with the name: " + scene_name_ +"\n");
                exit(0);
            }

            tracks_ = ISM::TracksPtr(new ISM::Tracks(recorded_object_sets));
        }


        void initVotes()
        {
            std::vector<std::string> pattern_names = table_helper_->getModelPatternNames();
            ISM::PatternToObjectToVoteMap pattern_to_object_to_votes;
            bool pattern_exist = false;

            // retrieve all votes for the pattern and sub-patterns
            for (std::string pattern_name : pattern_names)
            {
                if(pattern_name.find(scene_name_) == 0)
                {
                    std::set<std::pair<std::string, std::string> > objects;
                    objects =  table_helper_->getObjectTypesAndIdsBelongingToPattern(pattern_name);
                    pattern_to_object_to_votes[pattern_name] = table_helper_->getVoteSpecifiersForPatternAndObjects(pattern_name, objects);

                    pattern_exist = pattern_exist || !pattern_to_object_to_votes[pattern_name].empty();
                }
            }
            if (!pattern_exist)
            {
                ISM::printRed("No pattern in the database, with the name: " + scene_name_ + "\n");
                exit(0);
            }

            /* arrange votes by pattern and  trackIndex */
            for (std::pair<const std::string, ISM::ObjectToVoteMap>& object_to_votes : pattern_to_object_to_votes)
            {
                for (std::pair<const std::string, std::map<std::string, std::vector<ISM::VoteSpecifierPtr>>>& obj_id_to_votes : object_to_votes.second)
                {
                    for (std::pair<const std::string, std::vector<ISM::VoteSpecifierPtr>>& votes : obj_id_to_votes.second)
                    {
                        for(ISM::VoteSpecifierPtr vote : votes.second)
                        {
                            pattern_to_track_index_to_votes_[object_to_votes.first][vote->trackIndex].push_back(vote);
                        }
                    }
                }
            }
        }


        void initISMObjectTrack()
        {
            ISM::TracksPtr overall_tracks = ISM::TracksPtr(new ISM::Tracks(tracks_->tracks));
            ism_object_tracks_ = ISM::TracksPtr(new ISM::Tracks(std::vector<ISM::TrackPtr>()));

            // generate for each (sub-)pattern the corresponding reference track
            for (std::pair<const std::string, std::map<int, std::vector<ISM::VoteSpecifierPtr>>>& track_index_to_votes : pattern_to_track_index_to_votes_)
            {
                ISM::TrackPtr reference_track = ISM::TrackPtr(new ISM::Track(track_index_to_votes.first));

                // find reference object for each timestep of a track
                for (size_t i = 0; i < track_index_to_votes.second.size(); i++)
                {
                    // fill timesteps until now which we hadn't found a reference for, with (empty)object without type or id
                    while(reference_track->objects.size() < i)
                    {
                        reference_track->objects.push_back(ISM::ObjectPtr());
                    }

                    /* find reference object of the pattern and generate ism object  which represents this pattern */
                    for (ISM::VoteSpecifierPtr vote : track_index_to_votes.second[i])
                    {
                        if (ISM::GeometryHelper::isSelfVote(vote))
                        {
                            ISM::ObjectPtr ism_object;
                            ISM::TrackPtr track = overall_tracks->getTrackByTypeAndId(vote->objectType, vote->observedId);
                            if (track != nullptr)
                            {
                                ism_object = ISM::ObjectPtr(new ISM::Object(*(track->objects[i])));

                                if (ism_object != nullptr)
                                {
                                    ism_object->type = track_index_to_votes.first;
                                    ism_object->observedId = "";
                                }
                            }

                            reference_track->objects.push_back(ism_object);
                            break;
                        }
                    }
                }

                // fill timesteps after the last we found a reference for, with (empty)object without type or id
                while(reference_track->objects.size() < overall_tracks->tracks[0]->objects.size())
                {
                    reference_track->objects.push_back(ISM::ObjectPtr());
                }

                ism_object_tracks_->tracks.push_back(reference_track);
                overall_tracks->tracks.push_back(reference_track);
            }
        }

  
        void refreshVisualization(const ros::TimerEvent& e)
        {
            unsigned int previous_number = number_of_subscriber_;
            number_of_subscriber_ = visualization_publisher_.getNumSubscribers();
            if(number_of_subscriber_ > 0 && (number_of_subscriber_ > previous_number))
            {
                ISM::printGreen("Publish model marker!\n");
                model_visualizer_->publishCollectedMarkers();
                std::cout<<"\n";
            }
        }
  


        ros::NodeHandle nh_;
        ros::Timer timer_;
        ISM::TableHelperPtr table_helper_;

        std::string scene_name_;
        std::map<std::string, std::map<int, std::vector<ISM::VoteSpecifierPtr>>> pattern_to_track_index_to_votes_;
        ISM::TracksPtr tracks_;
        ISM::TracksPtr ism_object_tracks_;

        //Visualization stuff
        ros::Publisher visualization_publisher_;
        VIZ::ModelVisualizerRVIZ* model_visualizer_;
        unsigned int number_of_subscriber_;
};

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "modelViewer");
    ModelViewer* model_viewer = new ModelViewer();

    ros::spin();

    delete model_viewer;
    return 0;
}

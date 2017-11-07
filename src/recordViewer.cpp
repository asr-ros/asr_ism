/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Pkg includes
#include <ros/ros.h>

#include <asr_ism_visualizations/record_visualizer_rviz.hpp>
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>

//ISM includes
#include <ISM/common_type/Tracks.hpp>
#include <ISM/common_type/VoteSpecifier.hpp>
#include <ISM/utility/TableHelper.hpp>
#include <ISM/utility/Util.hpp>

class RecordViewer
{
    public:
        RecordViewer(): nh_("~")
        {
            std::string db_filename;
            std::string base_frame;
            std::string scene_name;
            std::string visualization_topic;

            getNodeParameters(db_filename, base_frame, scene_name, visualization_topic);
            ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(db_filename));
            if(!table_helper->recordDataExists())
            {
                ISM::printRed("The database \"" + db_filename + "\" doesn't contain any recordings!\n");
                exit(0);
            }

            std::vector<ISM::ObjectSetPtr> recordedObjectSets;

            ISM::RecordedPatternPtr recordedPattern = table_helper->getRecordedPattern(scene_name);
            if (recordedPattern == nullptr)
            {
                ISM::printRed("No pattern in the database, with the name: " + scene_name + "\n");
                exit(0);
            }

            recordedObjectSets = recordedPattern->objectSets;
            this->recorded_object_tracks_ = ISM::TracksPtr(new ISM::Tracks(recordedObjectSets));
            initRecordedObjects();

            visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 10000);
            record_visualizer_ = new VIZ::RecordVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);
            object_model_visualizer_ = new VIZ::ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);

            ISM::printGreen("Generate and publish record marker!\n");
            number_of_subscriber_ = visualization_publisher_.getNumSubscribers();
            object_model_visualizer_->drawObjectModels(recorded_objects_);
            record_visualizer_->addVisualization(recorded_object_tracks_);

            timer_ = nh_.createTimer(ros::Duration(1), &RecordViewer::refreshVisualization, this);
    }

    private:

        void getNodeParameters(std::string& db_filename,std::string& base_frame, std::string& scene_name, std::string& visualization_topic)
        {
            if (!nh_.getParam("dbfilename", db_filename)) {
                db_filename = "";
            }
            ROS_INFO_STREAM("dbfilename: " << db_filename);

            if (!nh_.getParam("baseFrame", base_frame)) {
                base_frame = "/map";
            }
            ROS_INFO_STREAM("baseFrame: " << base_frame);

            if (!nh_.getParam("visualization_topic", visualization_topic)) {
                visualization_topic = "";
            }
            ROS_INFO_STREAM("visualization_topic: " << visualization_topic);

            if (!nh_.getParam("sceneName", scene_name)) {
                scene_name = "";
            }
            ROS_INFO_STREAM("sceneName: " << scene_name);
        }


        void initRecordedObjects()
        {
            for(auto& track : recorded_object_tracks_->tracks)
            {
                for (auto obj : track->objects)
                {
                    if (obj)
                    {
                        recorded_objects_.push_back(obj);
                        break;
                    }
                }
            }
        }


        void refreshVisualization(const ros::TimerEvent& e)
        {
            unsigned int previous_number = number_of_subscriber_;
            number_of_subscriber_ = visualization_publisher_.getNumSubscribers();
            if(number_of_subscriber_ > 0 && (number_of_subscriber_ > previous_number))
            {                
                ISM::printGreen("Publish record marker!\n");
                record_visualizer_->publishCollectedMarkers();
                object_model_visualizer_->publishCollectedMarkers();
                std::cout<<"\n";
            }
        }


    private:
        ros::NodeHandle nh_;
        ros::Timer timer_;

        ISM::TracksPtr recorded_object_tracks_;
        std::vector<ISM::ObjectPtr> recorded_objects_;

        //Visualization stuff
        ros::Publisher visualization_publisher_;
        VIZ::RecordVisualizerRVIZ* record_visualizer_;
        VIZ::ObjectModelVisualizerRVIZ* object_model_visualizer_;
        unsigned int number_of_subscriber_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recordViewer");
    RecordViewer* record_viewer = new RecordViewer();

    ros::spin();

    delete record_viewer;
    return 0;
}

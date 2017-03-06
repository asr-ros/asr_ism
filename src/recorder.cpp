/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// Global includes
#include <string>

// ROS includes
#include <ros/ros.h>

// ISM includes
#include <ISM/recorder/Recorder.hpp>
#include <ISM/utility/Util.hpp>

// Local includes
#include "asr_ism/scene_configurator.hpp"


class Recorder
{
    public:
        Recorder() : nh_("~")
        {
            std::string db_filename;
            getNodeParameters(db_filename);

            if (db_filename != "")
            {
                ism_recorder_ = ISM::RecorderPtr(new ISM::Recorder(db_filename));
            }
            else
            {
                throw std::runtime_error("No db specified");
            }
        }


        void recordObjects(ISM::ObjectSetPtr object_set)
        {
            ISM::printYellow("Record object set:\n");

            ism_recorder_->insert(object_set, scene_name_);

            return;
        }


    private:
        ros::NodeHandle nh_;
        ISM::RecorderPtr ism_recorder_;
        std::string scene_name_;


        void getNodeParameters(std::string& db_filename)
        {
            if (!nh_.getParam("sceneName", scene_name_))
            {
                scene_name_ = "scene";
            }
            ROS_INFO_STREAM("sceneName: " << scene_name_);


            if (!nh_.getParam("dbfilename", db_filename))
            {
                db_filename = "record.sqlite";
            }
            ROS_INFO_STREAM("dbfilename: " << db_filename);
        }
};

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "recorder");
    Recorder* recorder = new Recorder();
    SceneConfigurator* scene_configurator = new SceneConfigurator(boost::bind(&Recorder::recordObjects, recorder, _1));

    ros::spin();

    //clean after shutdown
    delete scene_configurator;
    delete recorder;


    return 0;
}

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



//Pkg includes
#include <ros/ros.h>


//ISM includes
#include <ISM/tools/DataMerger.hpp>


bool getNodeParameters(ros::NodeHandle nh, std::string& target_file, std::vector<std::string>& source_files, bool& merge_records, bool& merge_models)
{
    bool success = true;

    if (!nh.getParam("target", target_file) || target_file.empty())
    {
        ROS_INFO("Missing parameter: \"target\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("target: " << target_file);
    }

    if (!nh.getParam("sources", source_files) || source_files.empty())
    {
        ROS_INFO("Missing parameter: \"sources\"");
        success = false;
    }
    else
    {
        std::stringstream info;
        info << "sources:";
        for (const std::string& file : source_files)
        {
            info << std::endl << "\t" << file;
        }
        ROS_INFO_STREAM(info.str());
    }

    if (!nh.getParam("merge_records", merge_records))
    {
        ROS_INFO("Missing parameter: \"merge_records\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("merge_records: " << merge_records);
    }

    if (!nh.getParam("merge_models", merge_models))
    {
        ROS_INFO("Missing parameter: \"merge_models\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("merge_models: " << merge_models);
    }


    return success;
}

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "data_merger");
    ros::NodeHandle nh("~");
    ISM::DataMerger data_merger;

    std::string target_file;
    std::vector<std::string> source_files;
    bool merge_records;
    bool merge_models;

    if(getNodeParameters(nh, target_file, source_files, merge_records, merge_models))
    {
        data_merger.merge(target_file, source_files, merge_records, merge_models);
        ROS_INFO("MERGE COMPLETED!");
    }
    else
    {
        ROS_INFO("MERGE ABORTED! Check launch-file for missing parameter.");
    }

    return 0;
}

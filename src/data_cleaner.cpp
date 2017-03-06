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
#include <ISM/tools/DataCleaner.hpp>


bool getNodeParameters(ros::NodeHandle nh, std::vector<std::string>& database_files, bool& clean_records, bool& clean_models)
{
    bool success = true;

    if (!nh.getParam("databases", database_files) || database_files.empty())
    {
        ROS_INFO("Missing parameter: \"databases\"");
        success = false;
    }
    else
    {
        std::stringstream info;
        info << "databases:";
        for (const std::string& file : database_files)
        {
            info << std::endl << "\t" << file;
        }
        ROS_INFO_STREAM(info.str());
    }   

    if (!nh.getParam("clean_records", clean_records))
    {
        ROS_INFO("Missing parameter: \"clean_records\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("clean_records: " << clean_records);
    }

    if (!nh.getParam("clean_models", clean_models))
    {
        ROS_INFO("Missing parameter: \"clean_models\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("clean_models: " << clean_models);
    }


    return success;
}

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "data_cleaner");
    ros::NodeHandle nh("~");
    ISM::DataCleaner data_cleaner;

    std::vector<std::string> database_files;
    bool clean_records;
    bool clean_models;

    if(getNodeParameters(nh, database_files, clean_records, clean_models))
    {
        if (clean_records)
        {
            data_cleaner.cleanRecords(database_files);
        }

        if (clean_models)
        {
            data_cleaner.cleanModels(database_files);
        }
        ROS_INFO("DATA CLEANING COMPLETED!");
    }
    else
    {
        ROS_INFO("DATA CLEANING ABORTED! Check launch-file for missing parameter.");
    }

    return 0;
}

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
#include <ISM/tools/RotationInvariantObjectsRotator.hpp>


bool getNodeParameters(ros::NodeHandle nh, std::string& source_file, std::string& target_file, std::string& object_type, std::string& object_id,
                       std::vector<std::string>& rotation_invariant_types)
{
    bool success = true;

    if (!nh.getParam("source", source_file) || source_file.empty())
    {
        ROS_INFO("Missing parameter: \"source\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("source: " << source_file);
    }

    if (!nh.getParam("target", target_file) || target_file.empty())
    {
        ROS_INFO("Missing parameter: \"target\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("target: " << target_file);
    }

    if (!nh.getParam("object_type", object_type))
    {
        ROS_INFO("Missing parameter: \"object_type\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("object_type: " << object_type);
    }

    if (!nh.getParam("object_id", object_id))
    {
        ROS_INFO("Missing parameter: \"object_id\"");
        success = false;
    }
    else
    {
        ROS_INFO_STREAM("object_id: " << object_id);
    }

    if (!nh.getParam("rotation_invariant_types", rotation_invariant_types) || rotation_invariant_types.empty())
    {
        ROS_INFO("Missing parameter: \"rotation_invariant_types\"");
        success = false;
    }
    else
    {
        std::stringstream info;
        info << "rotation_invariant_types:";
        for (const std::string& types : rotation_invariant_types)
        {
            info << std::endl << "\t" << types;
        }
        ROS_INFO_STREAM(info.str());
    }


    return success;
}

int main (int argc, char **argv)
{
    //Usual ros node stuff
    ros::init(argc, argv, "rotation_invariant_objects_rotator");
    ros::NodeHandle nh("~");
    ISM::RotationInvariantObjectsRotator rotator;

    std::string source_file;
    std::string target_file;
    std::string object_type;
    std::string object_id;
    std::vector<std::string> rotation_invariant_types;

    if(getNodeParameters(nh, source_file, target_file, object_type, object_id, rotation_invariant_types))
    {
        rotator.rotateRotationInvariantObjects(source_file, target_file, object_type, object_id, (object_type.empty() || object_id.empty()), rotation_invariant_types);
        ROS_INFO("ROTATION COMPLETED!");
    }
    else
    {
        ROS_INFO("ROTATION ABORTED! Check launch-file for missing parameter.");
    }

    return 0;
}

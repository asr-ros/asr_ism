/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// Local Includes
#include "asr_ism/scene_configurator.hpp"

// Global Includes
#include <iostream>

// ISM Includes
#include <ISM/utility/Util.hpp>



/****** Keyboard input includes and defines******/
#include <unistd.h>     //STDIN_FILENO

#define KEYCODE_AUTOMATIC_PROCESSING 0x20       //space
#define KEYCODE_PROCESS_ONLY_MARKED 0x6D        //m

#define KEYCODE_PROCESS_CONFIGURATION 0x0A      //enter

#define KEYCODE_RESET 0x7F          //delete or backspace
#define KEYCODE_ADD 0x61            //a
#define KEYCODE_SHOW 0x73           //s
#define KEYCODE_REMOVE 0x64         //d
#define KEYCODE_UPDATE_VIEW 0x75    //u

#define KEYCODE_HELP 0x68     //h
/****************************/

SceneConfigurator::SceneConfigurator(ProcessConfigurationFunction processConfigFunc, ProcessKeyboardInputFunction processKeyboardInputFunc, ProcessObjectInputFunction processObjectInputFunc)
    : processKeyboardInputFunc_(processKeyboardInputFunc), processObjectInputFunc_(processObjectInputFunc)
{
    nh_ = ros::NodeHandle(ros::this_node::getName(), "scene_configurator");

    assert(!processConfigFunc.empty());
    processConfigurationFunc_ = processConfigFunc;
    ROS_DEBUG_STREAM("processKeyboardInputFunc_ is empty: " << std::boolalpha << processKeyboardInputFunc_.empty());

    /* Required parameter from ROS environment */
    std::string object_input_topic;
    int object_input_queue_size;
    int object_input_thread_count;
    bool ignore_ids, ignore_types;
    std::string base_frame;
    bool use_confidence_from_msg;
    int enable_rotation_mode;
    std::string rotation_frame, rotation_object_type, rotation_object_id;
    bool enable_neighborhood_evaluation;
    double neighbour_angle_threshold, neighbour_distance_threshold;
    double automatic_processing_interval;
    int keyboard_poll_rate;
    std::string input_visualization_topic;

    getNodeParameters(object_input_topic, object_input_queue_size, object_input_thread_count, base_frame, ignore_types, ignore_ids, use_confidence_from_msg,
                      enable_rotation_mode, rotation_frame, rotation_object_type, rotation_object_id,
                      enable_neighborhood_evaluation, neighbour_angle_threshold, neighbour_distance_threshold,
                      keyboard_poll_rate, automatic_processing_interval, input_visualization_topic);


    /* default-state */
    automatic_processing_active_ = false;
    use_only_marked_objects_ = false;

    automatic_processing_timer_ = nh_.createTimer(ros::Duration(automatic_processing_interval), &SceneConfigurator::automaticProcessingCallback, this, false, false);

    /* Set-up keyboard input */
    tcgetattr( STDIN_FILENO, &original_terminal_settings_);           // save original terminal settings
    modified_terminal_settings_ = original_terminal_settings_;
    modified_terminal_settings_.c_lflag &= ~(ICANON | ECHO);          // disable buffering
    modified_terminal_settings_.c_iflag |= IUCLC;                     // map upper case to lower case
    modified_terminal_settings_.c_cc[VMIN] = 0;                       // minimal characters awaited
    modified_terminal_settings_.c_cc[VTIME] = 0;                      // time keep reading after last byte
    tcsetattr( STDIN_FILENO, TCSANOW, &modified_terminal_settings_);  // apply modified terminal settings

    //Set-up thread to poll keyboard input
    keyboard_thread_ = new boost::thread(&SceneConfigurator::keyboardInputMain, this, keyboard_poll_rate);


    /* Set-up object input */
    object_msg_converter_ = IH::ObjectConverterPtr(new IH::ObjectConverter(base_frame, ignore_types, ignore_ids, enable_rotation_mode, rotation_frame, rotation_object_type, rotation_object_id, use_confidence_from_msg));
    data_ = new SceneConfiguratorData(enable_neighborhood_evaluation, neighbour_angle_threshold, neighbour_distance_threshold);
    ptu_ = ros::NodeHandlePtr(new ros::NodeHandle("asr_flir_ptu_driver"));
    ros::NodeHandle object_input_nh_ = ros::NodeHandle(nh_, "object_input");
    object_input_nh_.setCallbackQueue(&object_queue_);
    object_input_subscriber_ = object_input_nh_.subscribe(object_input_topic, object_input_queue_size, &SceneConfigurator::objectInputCallback, this);
    spinner_ = new ros::AsyncSpinner(object_input_thread_count, &object_queue_);
    spinner_->start();


    /* Set-up visualization */
    visualization_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(input_visualization_topic, 10000);
    object_model_visualizer_ = new VIZ::ObjectModelVisualizerRVIZ(visualization_publisher_, base_frame, "", 0);
    visualization_timer_ = nh_.createTimer(ros::Duration(std::max(automatic_processing_interval, 0.5)), &SceneConfigurator::updateVisualizationCallback, this);

    printHelpText();
}

SceneConfigurator::~SceneConfigurator()
{
    ROS_DEBUG("~SceneConfigurator():");
    //Stop keyboard-thread and restore terminal settings
    keyboard_thread_->interrupt();
    keyboard_thread_->join();
    tcsetattr( STDIN_FILENO, TCSANOW, &original_terminal_settings_);  // restore original terminal settings
    ROS_DEBUG("Terminal Settings restored");


    //Stop object input handling
    spinner_->stop();
    delete spinner_;

    delete object_model_visualizer_;
}

bool SceneConfigurator::getAutomaticProcessingActive()
{
    return this->automatic_processing_active_;
}

void SceneConfigurator::keyboardInputMain(const unsigned int poll_rate)
{
    ros::Rate rate(poll_rate);
    try
    {
        while (ros::ok())
        {
            boost::this_thread::interruption_point();

            processKeyboardInput();

            rate.sleep();
        }
    }
    catch (boost::thread_interrupted&) {}

    return;
}

void SceneConfigurator::processKeyboardInput()
{
    int temp_key_value = getchar();
    tcflush(STDIN_FILENO, TCIFLUSH);  //flush unread input, because we don't want to process old input from buffer.

    // don't accept repeated key input if a key is still pressed.
    bool old_key_pressed = key_pressed_;
    key_pressed_ = (temp_key_value < 0) ? false : true;
    if(!key_pressed_ || old_key_pressed)
    {
        return;
    }


    // commands to change state, and print help info.
    switch(temp_key_value)
    {
        case KEYCODE_PROCESS_ONLY_MARKED :
            use_only_marked_objects_ = !use_only_marked_objects_;
            printStatus();
            return;
        case KEYCODE_AUTOMATIC_PROCESSING:
            automatic_processing_active_ = !automatic_processing_active_;
            if (automatic_processing_active_)
            {
                automatic_processing_timer_.start();
            }
            else
            {
                automatic_processing_timer_.stop();
            }

            printStatus();
            return;
        case KEYCODE_HELP:
            printHelpText();
            // No return so the key will be passed (processKeyboardInputFunc_) for additional help text.
    }

    // commands for configurator with disabled auto capturing
    if(!automatic_processing_active_)
    {
        switch(temp_key_value)
        {
            case KEYCODE_RESET:
                ISM::printRed("\tRESET\n");
                data_->clearAllEstimations();
                break;
            case KEYCODE_ADD:
                ISM::printBlue("\tADD\n");
                data_->setMarkedForAllEstimationsInView(true);
                break;
            case KEYCODE_REMOVE:
                ISM::printBlue("\tREMOVE\n");
                data_->setMarkedForAllEstimationsInView(false);
                break;
            case KEYCODE_PROCESS_CONFIGURATION:
                ISM::printGreen("\tPROCESS_CONFIGURATION!\n");
                processConfiguration();
                break;
            case KEYCODE_SHOW:
                ISM::printBlue("\tSHOW_CONFIGURATION\n");
                printEstimations();
                break;
            case KEYCODE_UPDATE_VIEW:
                ISM::printBlue("\tUPDATE_VIEW\n");
                data_->clearUnmarkedEstimations();
                data_->resetInViewForAllEstimations();
                break;
            default:
                if(processKeyboardInputFunc_)
                {
                    processKeyboardInputFunc_(temp_key_value);
                }
                break;
        }
    }

    return;
}

void SceneConfigurator::objectInputCallback(const asr_msgs::AsrObject &object)
{
    ROS_DEBUG_STREAM("Received object message. THREAD-ID: " << boost::this_thread::get_id());

    //Do not capture and transform objects while CAMERA is moving around.
    if (isCameraInMotion())
    {
        ROS_DEBUG("Rejected object message and clean estimations since camera is currently moving ");

        data_->clearUnmarkedEstimations();
        data_->resetInViewForAllEstimations();
        return;
    }

    try
    {
        ISM::ObjectPtr obj = object_msg_converter_->pbdObjectToISMObject(object);

        data_->addEstimation(obj);

        if(processObjectInputFunc_)
        {
            processObjectInputFunc_(obj);
        }
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

bool SceneConfigurator::isCameraInMotion()
{
    bool isInMotion;

    // is PTU moving?
    bool stopped = false;
    bool retrieved = ptu_->getParam("reached_desired_position", stopped);
    isInMotion = !stopped && retrieved;

    return isInMotion;
}

void SceneConfigurator::processConfiguration()
{    
    ISM::ObjectSetPtr set = use_only_marked_objects_ ? data_->getObjectSetFromMarkedEstimations() : data_->getObjectSetFromAllEstimations();
    if (set->objects.empty())
    {
        ISM::printRed("Configuration is empty!\n");
    }
    else
    {
        //Normalize rotation invarince objects, e.g. Cup, PlateDeep
        object_msg_converter_->normalizeRotationInvariantObjects(set);

        processConfigurationFunc_(set);
    }
}

void SceneConfigurator::automaticProcessingCallback(const ros::TimerEvent &e)
{
    automatic_processing_timer_.stop();
    ISM::printGreen("\tAUTO_PROCESS_CONFIGURATION!\n");
    processConfiguration();

    // reset values to use only use objects that are still there
    data_->clearUnmarkedEstimations();
    data_->resetInViewForAllEstimations();
    automatic_processing_timer_.start();
}

void SceneConfigurator::printEstimations()
{
    ObjectEstimationPtrs estimations = data_->getAllEstimations();
    ISM::printGreen("Marked and in view:\n");
    for (const ObjectEstimationPtr est : estimations)
    {
        if (est->marked && est->in_view)
        {
            std::cout << "\t" << est->object->type << " " << est->object->observedId << std::endl;
        }
    }
    ISM::printBlue("Marked and out of view:\n");
    for (const ObjectEstimationPtr est : estimations)
    {
        if (est->marked && !est->in_view)
        {
            std::cout << "\t" << est->object->type << " " << est->object->observedId << std::endl;
        }
    }
    ISM::printYellow("Not marked:\n");
    for (const ObjectEstimationPtr est : estimations)
    {
        if (!est->marked)
        {
            std::cout << "\t" << est->object->type << " " << est->object->observedId << std::endl;
        }
    }
    std::cout << std::endl;
}

void SceneConfigurator::printHelpText()
{
    std::stringstream commands;
    if (automatic_processing_active_)
    {
        commands << "Commands for Auto Processing:\n"
                 << "\tpress \"m\"\tchange PROCESSING_MODE\n"
                 << "\tpress \"space\"\tswitch to Manual Processing\n\n"
                 << "\tpress \"h\"\tshow INFO and COMMAND description\n"
                 << "\tpress \"^C\"\tquit node\n";
    }
    else
    {
         commands << "Commands for Manual Processing:\n"
                  << "\tpress \"m\"\t-change PROCESSING_MODE\n"
                  << "\tpress \"space\"\t-switch to Auto Processing\n\n"
                  << "\tpress \"enter\"\t-process object configuration\n"
                  << "\tpress \"delete\\\t-reset all objects\n\t      backspace\"\n\n"
                  << "\tpress \"a\"\t-add objects from current view to marked objects\n"
                  << "\tpress \"d\"\t-remove objects in the current view from marked objects\n"
                  << "\tpress \"u\"\t-update view to get rid of old estimations\n\n"
                  << "\tpress \"s\"\t-show object configuration\n\n"
                  << "\tpress \"h\"\t-show INFO and COMMAND description\n"
                  << "\tpress \"^C\"\t-quit node\n";
    }

    printStatus();
    ISM::printYellow(commands.str() + "\n");

    return;
}

void SceneConfigurator::printStatus()
{
    std::stringstream status, info;

    info << "rviz color legend: ";

    if (automatic_processing_active_)
    {
        status << "STATUS: Auto Processing";
    }
    else
    {
        status << "STATUS: Manual Processing";
    }

    status << "\t\tPROCESSING_MODE: ";

    if(use_only_marked_objects_)
    {
        status << "Only Marked Objects\n";

        info << "\033[32;1m" << "marked and in current view (green), "
             << "\033[36m" << "marked but not in current view (blue)."
             << "\033[0m" << "\n";
    }
    else
    {
        status << "All Objects\n";

        info << "\033[32;1m" << "marked and in current view (green), "
             << "\033[36m" << "marked but not in current view (blue), "
             << "\033[33m" << "not marked but in the current view (yellow)."
             << "\033[0m" << "\n";
    }

    ISM::printGreen(status.str());
    std::cout << "\n" << info.str() << "\n";

    return;
}

void SceneConfigurator::updateVisualizationCallback(const ros::TimerEvent &e)
{
    if (visualization_publisher_.getNumSubscribers() > 0)
    {
        ROS_DEBUG("Publish marker");

        ObjectEstimationPtrs estimations = data_->getAllEstimations();

        std::vector<ISM::ObjectPtr> objects;
        std::map<ISM::ObjectPtr, double> objects_to_hue_map;

        for (const ObjectEstimationPtr est : estimations)
        {
            double hueTemp;
            bool visualize = false;
            if (est->marked)
            {
                hueTemp = est->in_view ? 120.0 : 180.0;
                visualize = true;
            }
            else if (!use_only_marked_objects_ && est->in_view)
            {
                hueTemp = 60.0;
                visualize = true;
            }

            if (visualize)
            {
                objects.push_back(est->object);
                objects_to_hue_map.insert(std::make_pair(est->object, hueTemp));
            }
        }
        object_model_visualizer_->clearAllMarkerOfTopic();
        object_model_visualizer_->drawObjectModels(objects, objects_to_hue_map);
    }
    else
    {
        ROS_DEBUG("No subscriber exist for the visualization.\n");
    }
}

void SceneConfigurator::getNodeParameters(std::string& object_topic, int& queue_size, int& object_input_thread_count, std::string& base_frame, bool& ignore_types, bool& ignore_ids, bool& use_confidence_from_msg,
                                          int& enable_rotation_mode, std::string& rotation_frame, std::string& rotation_object_type, std::string& rotation_object_id,
                                          bool& enable_neighborhood_evaluation, double& neighbour_angle_threshold, double& neighbour_distance_threshold,
                                          int& keyboard_poll_rate, double& automatic_processing_interval, std::string& input_visualization_topic)
{
    // NodeHandle with the parent namespace (e.g. parent = recorder or recognizer), because the parameters are prefixed with this namespace.
    ros::NodeHandle nh = ros::NodeHandle(ros::this_node::getName());

    if (!nh.getParam("objectTopic", object_topic))
    {
        object_topic = "/objects";
    }
    ROS_INFO_STREAM("objectTopic: " << object_topic);

    if (!nh.getParam("queueSizePbdObjectCallback", queue_size))
    {
        queue_size = 100;
    }
    ROS_INFO_STREAM("queueSizePbdObjectCallback: " << queue_size);

    if (!nh.getParam("object_input_thread_count", object_input_thread_count))
    {
        object_input_thread_count = 1;
    }
    ROS_INFO_STREAM("object_input_thread_count: " << object_input_thread_count);

    if (!nh.getParam("baseFrame", base_frame))
    {
        base_frame = "/map";
    }
    ROS_INFO_STREAM("baseFrame: " << base_frame);

    if (!nh.getParam("use_confidence_from_msg", use_confidence_from_msg))
    {
        use_confidence_from_msg = false;
    }
    ROS_INFO_STREAM("use_confidence_from_msg: " << use_confidence_from_msg);

    if (!nh.getParam("enableRotationMode", enable_rotation_mode))
    {
        enable_rotation_mode = 0;
    }
    ROS_INFO_STREAM("enableRotationMode: " << enable_rotation_mode);

    if (!nh.getParam("rotationFrame", rotation_frame))
    {
        rotation_frame = "/map";
    }
    ROS_INFO_STREAM("rotationFrame: " << rotation_frame);

    if (!nh.getParam("rotationObjectType", rotation_object_type))
    {
        rotation_object_type = "";
    }
    ROS_INFO_STREAM("rotationObjectType: " << rotation_object_type);

    if (!nh.getParam("rotationObjectId", rotation_object_id))
    {
        rotation_object_id = "";
    }
    ROS_INFO_STREAM("rotationObjectId: " << rotation_object_id);

    if (!nh.getParam("enableNeighborhoodEvaluation", enable_neighborhood_evaluation))
    {
        enable_neighborhood_evaluation = false;
    }
    ROS_INFO_STREAM("enableNeighborhoodEvaluation: " << enable_neighborhood_evaluation);

    if (!nh.getParam("angleNeighbourThreshold", neighbour_angle_threshold))
    {
        neighbour_angle_threshold = 30;
    }
    ROS_INFO_STREAM("angleNeighbourThreshold: " << neighbour_angle_threshold);

    if (!nh.getParam("distanceNeighbourThreshold", neighbour_distance_threshold))
    {
        neighbour_distance_threshold = 0.05;
    }
    ROS_INFO_STREAM("distanceNeighbourThreshold: " << neighbour_distance_threshold);

    if (!nh.getParam("keyboard_poll_rate", keyboard_poll_rate))
    {
        keyboard_poll_rate = 10;
    }
    ROS_INFO_STREAM("keyboard_poll_rate: " << keyboard_poll_rate);

    if (!nh.getParam("capture_interval", automatic_processing_interval))
    {
        automatic_processing_interval = 1;
    }
    ROS_INFO_STREAM("captureInterval: " << automatic_processing_interval);

    if (!nh.getParam("input_objects_visualization_topic", input_visualization_topic))
    {
        input_visualization_topic = "input_objects_scene_configurator_viz";
    }
    ROS_INFO_STREAM("input_objects_visualization_topic: " << input_visualization_topic);

    if (!nh.getParam("ignoreTypes", ignore_types))
    {
        ignore_types = false;
    }
    ROS_INFO_STREAM("ignoreTypes: " << ignore_types);

    if (!nh.getParam("ignoreIds", ignore_ids))
    {
        ignore_ids = false;
    }
    ROS_INFO_STREAM("ignoreIds: " << ignore_ids);
}

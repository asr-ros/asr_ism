/**

Copyright (c) 2016, Borella Jocelyn, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Local Includes
#include "scene_configurator_data.hpp"
#include "../ism_helper.hpp"

// Global Includes
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <termios.h>    //Keyboard Input: termios, TCSANOW, ECHO, ICANON

// Pkg Includes
#include "ros/ros.h"
#include <asr_ism_visualizations/ObjectModelVisualizerRVIZ.hpp>
#include <asr_msgs/AsrObject.h>

// ISM Inlcude
#include <ISM/common_type/Object.hpp>


/**
 *  SceneConfigurator class manages scene configurations and handles object & keyboard input.
 */
class SceneConfigurator
{
    public:
        typedef boost::function<void(ISM::ObjectSetPtr)> ProcessConfigurationFunction;
        typedef boost::function<void(int)> ProcessKeyboardInputFunction;
        typedef boost::function<void(ISM::ObjectPtr)> ProcessObjectInputFunction;


        /**
         * SceneConfigurator constructor.
         *
         * @param processConfigFunc Function to process an object configuration.
         * @param processKeyboardInputFunc Function to process further keyboard commands.
         * @param processObjectInputFunc Function for further processing of incoming objects.
         */
        SceneConfigurator(ProcessConfigurationFunction processConfigFunc, ProcessKeyboardInputFunction processKeyboardInputFunc = ProcessKeyboardInputFunction(), ProcessObjectInputFunction processObjectInputFunc = ProcessObjectInputFunction());
        ~SceneConfigurator();

        /**
         * @return if object configurations are processed automatically in a certain time interval.
         */
        bool getAutomaticProcessingActive();

    private:
        ros::NodeHandle nh_;

        IH::ObjectConverterPtr object_msg_converter_;
        SceneConfiguratorData* data_;
        ros::Timer automatic_processing_timer_;

        /*** Functions provided from other objects ***/
        ProcessConfigurationFunction processConfigurationFunc_;
        ProcessKeyboardInputFunction processKeyboardInputFunc_;
        ProcessObjectInputFunction processObjectInputFunc_;

        /*** States ***/
        //Whether only marked objects or marked objects and any additional object in the current view are used for scene processing.
        bool use_only_marked_objects_;
        //Whether to process the scene automatically.
        bool automatic_processing_active_;

        /*** Keyboard Input ***/
        boost::thread *keyboard_thread_;
        termios original_terminal_settings_;
        termios modified_terminal_settings_;
        bool key_pressed_;

        /*** Object Input ***/
        ros::CallbackQueue object_queue_;
        ros::Subscriber object_input_subscriber_;
        ros::AsyncSpinner *spinner_;
        ros::NodeHandlePtr ptu_;

        /*** Visualization ***/
        ros::Publisher visualization_publisher_;
        VIZ::ObjectModelVisualizerRVIZ* object_model_visualizer_;
        ros::Timer visualization_timer_;



        /**
         * Main-Function of keyboard thread.
         *
         * @param poll_rate Rate (Hz) in which the keyboard will be polled for input.
         */
        void keyboardInputMain(const unsigned int poll_rate);
        void processKeyboardInput();


        /**
         * Extract class and identifier (within class) from estimation given by object localization system. Object 6D pose is extracted as well and transformed into world coordinate frame fixed during app startup.
         *
         * @param object ROS object msgs from which information is extracted.
         */
        void objectInputCallback(const asr_msgs::AsrObject& object);

        /**
         * Check if the camera is in any kind of motion, e.g. the robot is moving.
         */
        bool isCameraInMotion();

        /**
         * Wrapper for the passed function processConfigFunc.
         */
        void processConfiguration();
        
        /**
         * Callback for automatic processing of configurations.
         *
         * @param e Timer event with no meaning to us beyond its existence.
         */
        void automaticProcessingCallback(const ros::TimerEvent& e);

        /* Print Information on CLI */
        void printEstimations();
        void printHelpText();
        void printStatus();

        /**
         * Callback to update visualization.
         *
         * @param e Timer event with no meaning to us beyond its existence.
         */
        void updateVisualizationCallback(const ros::TimerEvent& e);

        /**
         * Gets parameters from ROS Parameter Server.
         *
         * @param object_topic ROS topic on which object estimation messages are received.
         * @param object_input_queue_size Size of the ros::CallbackQueue for incoming object estimation messages.
         * @param object_input_thread_count Number of threads to process incoming object estimations.
         * @param base_frame Frame to which incoming object messages are transformed.
         * @param ignore_types Whether to use the type from AsrObject for converted ism object type.
         * @param ignore_ids Whether to use the id from AsrObject for converted ism object id.
         * @param use_confidence_from_msg Whether to use confidence from incoming object or use confidence 1.0.
         * @param enable_rotation_mode Whether incoming object orientations should be rotated to the rotation_frame (The rotation only takes place if baseFrame == rotationFrame.) or to an object.
         * @param rotation_frame
         * @param rotation_object_type
         * @param rotation_object_id
         * @param enable_neighborhood_evaluation Whether to use neightborhood evaluation or not.
         * @param neighbour_angle_threshold
         * @param neighbour_distance_threshold
         * @param keyboard_poll_rate
         * @param visualization_topic Where to publish visualization of scene configuration.
         */
        void getNodeParameters(std::string& object_topic, int& queue_size, int& object_input_thread_count, std::string& base_frame, bool& ignore_types, bool& ignore_ids, bool& use_confidence_from_msg,
                               int& enable_rotation_mode, std::string& rotation_frame, std::string& rotation_object_type, std::string& rotation_object_id,
                               bool& enable_neighborhood_evaluation, double& neighbour_angle_threshold, double& neighbour_distance_threshold,
                               int& keyboard_poll_rate, double& automatic_processing_interval, std::string& visualization_topic);
};

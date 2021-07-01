/*!*******************************************************************************************
 *  \file       quadrotor_pid_controller_process.h
 *  \brief      Quadrotor Pid Controller implementation file.
 *  \details    The ROS node "Quadrotor PID controller" implements a motion controller for multirotors that operates basically in the following way:
 *              The controller receives as input data a motion reference that establishes the movement goal expressed as follows: 
 *              the desired pose <x, y, z, yaw>
 *              or the desired ground speed <dx, dy>.
 *              As a result, the controller generates a periodic output at a high frequency for the multirotor actuators. 
 *              This output is expressed with the following values <yaw_rate, altitud_rate, pitch, roll>.
 *  \authors    Alberto Rodelgo Perales
 *              Pablo Santofimia Ruiz
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/ 


#ifndef quadrotor_controller_process
#define quadrotor_controller_process

#include <string>
#include <robot_process.h>
#include "ros/ros.h"

#include "yaw_controller.h"
#include "altitude_controller.h"
#include "position_controller.h"
#include "speed_controller.h"
//Msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mav_msgs/RollPitchYawrateThrust.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/MotionControlMode.h"
#include <geometry_msgs/Vector3.h>
#include "xmlfilereader.h"
#include <yaml-cpp/yaml.h>
#include "cvg_string_conversions.h"

class QuadrotorControllerProcess : public RobotProcess
{
public:
    QuadrotorControllerProcess();
    ~QuadrotorControllerProcess();
    bool isStarted();
    double get_moduleRate();  
protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();

    AltitudeController altitude_control;
    YawController yaw_control;
    PositionController position_control;
    SpeedController speed_control;
    std::string robot_config_path;

    double rate;
    bool started;
    std::string robot_namespace;
    std::string config_file;
    void publishControllerReferences();

    //Topics
    std::string self_localization_pose_topic_name;
    std::string self_localization_speed_topic_name;
    std::string motion_reference_pose_topic_name;
    std::string motion_reference_ground_speed_topic_name;
    std::string actuator_command_roll_pitch_topic_name;
    std::string actuator_command_altitude_yaw_topic_name;
    std::string motion_reference_assumed_control_mode_topic_name;
    std::string motion_reference_assumed_ground_speed;
    std::string motion_reference_assumed_pose_topic_name;

    //Subscribers
    ros::Subscriber selfLocalizationPose;
    ros::Subscriber selfLocalizationSpeed;
    ros::Subscriber motionReferencePose;
    ros::Subscriber motionReferenceGroundSpeed;
    void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void motionReferencePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void motionReferenceGroundSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    //Publishers
    ros::Publisher actuator_pitch_roll_pub;
    ros::Publisher actuator_altitude_yaw_pub;
    ros::Publisher assumedControlMode;
    ros::Publisher assumedGroundSpeed;
    ros::Publisher assumedPose;

    // Antonio Publishers
    ros::Publisher actuator_command_pose;
    ros::Publisher actuator_command_speed;

    //Variables
    geometry_msgs::PoseStamped self_localization_pose;
    geometry_msgs::PoseStamped assumed_pose;
    geometry_msgs::Vector3 self_localization_angular_euler;    
    geometry_msgs::TwistStamped current_speed;
    geometry_msgs::PoseStamped motion_reference_pose;
    geometry_msgs::TwistStamped motion_reference_ground_speed;
    /////////////////////mav_msgs::RollPitchYawrateThrust actuator_command;
    geometry_msgs::TwistStamped assumed_ground_speed;
    geometry_msgs::PoseStamped actuator_pitch_roll;
    geometry_msgs::TwistStamped actuator_altitude_yaw;

    // Antonio Variables
    geometry_msgs::PoseStamped command_pose;
    geometry_msgs::TwistStamped command_speed;

    //Quaternion to Euler
    geometry_msgs::Vector3 toEulerianAngle(geometry_msgs::PoseStamped q);

    int control_mode, init_control_mode;// Active and initial control modes
    inline int getControlMode() { return control_mode; }
    inline int getInitControlMode() { return init_control_mode; }

    int publishDroneNavCommand(void);
    void setNavCommand(float pitch, float roll, float dyaw, float dz, double time=-1.0);
    void setNavCommandToZero();
    void getOutput(float *pitch_val, float *roll_val, float *dyaw_val, float *dz_val);
    void setPositionControl();
    void setSpeed3DControl();
    void setSpeedControl();
    void setGroundSpeedControl();
    void setAttitudeControl();
    void resetActuatorCommands();

    //Control mode
    std::string set_control_mode_service_name;
    ros::ServiceServer set_control_mode_service;
    bool setControlMode_Serv_Call(aerostack_msgs::SetControlMode::Request& request, aerostack_msgs::SetControlMode::Response& response);
    bool setControlMode(int mode);
    void publishControlMode();
};

#endif // quadrotor_controller_process

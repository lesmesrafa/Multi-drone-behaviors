/*!********************************************************************************
 * \brief     Take off behavior implementation 
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
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
 *******************************************************************************/

#ifndef TAKE_OFF_WITH_PID_H
#define TAKE_OFF_WITH_PID_H

// System
#include <string>
#include <thread>
#include <tuple>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <fstream>
// ROS
#include "std_srvs/Empty.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/FlightActionCommand.h>
#include <sensor_msgs/BatteryState.h>
#include <aerostack_msgs/FlightState.h>
#include "aerostack_msgs/SetControlMode.h"

// Aerostack libraries
#include <BehaviorExecutionManager.h>
#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"

#define TAKEOFF_ALTITUDE 1.0f
#define TAKEOFF_SPEED 0.5f

#define LANDING_ALTITUDE -1.0f
#define LANDING_SPEED -0.5f

#define LANDING_TIMEOUT 60
#define LANDING_CHECK_DELAY 1
#define LANDED_Z_SPEED 0.05

#define DEBUG 0
#define USE_POSE_REFERENCES 1

class BehaviorLandWithPid : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorLandWithPid();
  ~BehaviorLandWithPid();
  int main(int argc, char** argv);

private:
  // ros::NodeHandle node_handle;

  ros::NodeHandle nh;
  std::string nspace;

  // Congfig variables
  std::string state_str;
  std::string battery_topic; 

	std::string estimated_speed_topic;
	std::string estimated_pose_topic;
	std::string flight_action_topic;
	std::string motion_reference_speed_topic;
	std::string motion_reference_pose_topic;
	std::string set_control_mode_service_name;
  std::string status_str;

  // Communication variables
  ros::Subscriber status_sub;
  ros::Subscriber battery_subscriber;

  ros::Publisher flight_action_pub;

  ros::Subscriber pose_sub_;
  ros::Subscriber speeds_sub_;
  ros::Subscriber flight_action_sub;
  ros::ServiceClient set_control_mode_client_srv_;
  ros::Publisher speed_references_pub_;
  ros::Publisher pose_references_pub_;
  ros::Publisher flightstate_pub;

  // Messages
  aerostack_msgs::FlightState status_msg;

  float dz_measure_;
  double roll_ = 0.0f, pitch_ = 0.0f;
  geometry_msgs::Point position_;
  aerostack_msgs::FlightActionCommand flight_action_msg_;
  ros::Time landing_command_time_;
  geometry_msgs::Point activationPosition;

private:
  // BehaviorExecutionManager
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  void checkDroneState();
  bool enableDroneControl(const int& mode);
  bool checkLanding();
  void sendAltitudeSpeedReferences(const double& dz_speed , const double takeoff_altitude = TAKEOFF_ALTITUDE);

public: // Callbacks
  void statusCallBack(const aerostack_msgs::FlightState &msg);
  void poseCallback(const geometry_msgs::PoseStamped&);
  void speedsCallback(const geometry_msgs::TwistStamped&);
  void flightActionCallback(const aerostack_msgs::FlightActionCommand& _msg);
};

#endif

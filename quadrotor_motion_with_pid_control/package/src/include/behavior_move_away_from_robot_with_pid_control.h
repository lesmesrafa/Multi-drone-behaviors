/*!********************************************************************************
 * \brief     behavior_move_away_from_robot_with_pid_control implementation
 * \authors   Rafael Martin Lesmes
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
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

#ifndef MOVE_AWAY_FROM_ROBOT_WITH_PID_CONTROL_H
#define MOVE_AWAY_FROM_ROBOT_WITH_PID_CONTROL_H

// System
#include <string>
#include "math.h"
#include <vector>
#include <stdlib.h>
// ROS
#include "std_srvs/Empty.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Aerostack msgs
#include <aerostack_msgs/FlightActionCommand.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "aerostack_msgs/FlightState.h"
#include <nav_msgs/Path.h>
#include <aerostack_msgs/SharedRobotPosition.h>
#include <std_msgs/Bool.h>
// Aerostack libraries
#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"
#include <BehaviorExecutionManager.h>
#include "aerostack_msgs/SetControlMode.h"
#include <yaml-cpp/yaml.h>

class BehaviorMoveAwayFromRobotWithPidControl : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorMoveAwayFromRobotWithPidControl();
  ~BehaviorMoveAwayFromRobotWithPidControl();

private:
  // Config variables
  ros::NodeHandle node_handle;
  std::string nspace; 

  std::string command_high_level_str;
  std::string motion_reference_pose_str;
  std::string self_localization_pose_str;  
  std::string motion_reference_speed_str;
  std::string status_str;
  std::string set_control_mode_srv;
  std::string shared_robot_position_channel_topic_str;
  // Communication variables 
  // Subscriber
  ros::Subscriber self_localization_pose_sub;   
  ros::Subscriber status_sub;
  ros::Subscriber shared_robot_pos_sub;
  //Publishers
  ros::Publisher command_high_level_pub;
  ros::Publisher motion_reference_speed_pub;
  ros::Publisher motion_reference_pose_pub;
  //Service
  ros::ServiceClient set_control_mode_client_srv;
  //Messages
  aerostack_msgs::FlightState status_msg;
  aerostack_msgs::FlightActionCommand high_level_command;
  geometry_msgs::TwistStamped motion_reference_speed;
  geometry_msgs::PoseStamped estimated_pose_msg;
  aerostack_msgs::SharedRobotPosition shared_drone_coord;
  geometry_msgs::PoseStamped path_point;
  tf2::Quaternion orientation_quaternion;
  // Other variables
  double distxyz, distx, disty, distz, distxy;
  std::string FOLLOWER_NAME;
  double SEPARATION_DISTANCE;
  double MAXIMUM_SPEED;
  std::string YAW;
  int time;
  bool estimated_pose_msg_received = false;
  bool shared_robot_pos_received = false;
  double pose_z_ini;
  

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

public:
  // Callbacks
  void selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg);
  void statusCallBack(const aerostack_msgs::FlightState &msg);
  void sharedRobotPositionCallback(const aerostack_msgs::SharedRobotPosition &msg);
  // Other functions
  bool setControlMode(int new_control_mode);
  void speedSelfRegulation();

};

#endif

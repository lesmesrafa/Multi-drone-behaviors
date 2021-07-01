/*!*******************************************************************************************
 *  \file       path_tracker_process.h
 *  \brief      Path Tracker implementation file.
 *  \details    The ROS node “path_tracker” implements an algorithm for path tracking for autonomous robots that operates in the following way:
 *              The node receives as input data a path to track as a sequence of point values. 
 *              As a result, the controller generates a periodic output for a motion controller with reference speed and reference.
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

#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <ros/ros.h>
#include <robot_process.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include "math.h"

//Finite state machine
const int MOVING_STRAIGHT = 1;
const int MOVING_AROUND = 2;
const int ENDING_MOVEMENT = 3;

class PathTrackerProcess : public RobotProcess
{

public:
    PathTrackerProcess();
    ~PathTrackerProcess();
    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
public:
  double get_moduleRate();

  //! Read Config
  bool readConfigs(std::string configFile);

private:
  int reference_point_index;
  double rate;

  //! ROS NodeHandler used to manage the ROS communication
  ros::NodeHandle nIn;

  //! Configuration file variable
  std::string robot_namespace;               // drone id namespace
  std::string config_file;    // Config File String name
  std::string robot_config_path;     // Config File String aerostack path name

  //! ROS publisher
  ros::Publisher motion_reference_speed;
  ros::Publisher motion_reference_pose;
  ros::Publisher motion_reference_remaining_path;

  //Subscribers
  ros::Subscriber self_localization_pose;
  ros::Subscriber self_localization_speed;
  ros::Subscriber motion_reference_path;

  // Speed Commanded
  float dx;
  float dy;
  float dz;

  // Constants
  float v_maxxy;
  float v_maxz;
  bool path_facing;
  float precision;

  //Distance to target
  double distance;
  float current_distance;

  // Trajectory
  geometry_msgs::TwistStamped speed_result;
  geometry_msgs::PoseStamped last_point;
  geometry_msgs::PoseStamped reference_point;
  geometry_msgs::PoseStamped next_point;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped fixed_orientation;
  nav_msgs::Path reference_path;  
  nav_msgs::Path remaining_path;
  float number_of_points;

  //FSM
  int flight_state;
  bool iterate;
  bool started;
  bool end_moving_around;
  
  float distx;
  float disty;
  float distz;  
  float distxy;
  float distxyz;

  float next_distx;
  float next_disty;
  float next_distz;
  float next_distxy;
  float next_distxyz;

  double t_turn;
  float t;
  double time;

  float vx;
  float vy;
  float vz;  
  float vxy;

  float next_vx;
  float next_vy;
  float next_vz;  
  float next_vxy;

  float point_to_distance_x1;
  float point_to_distance_y1;
  float point_to_distance_z1;  
  float point_to_distance_x2;
  float point_to_distance_y2;
  float point_to_distance_z2;  
  float perp_vector_x1;
  float perp_vector_y1;
  float perp_vector_x2;
  float perp_vector_y2;  
  float circumference_x;
  float circumference_y;
  float angle;
  float r;
  float arch_lenght;
  
  //Topics
  std::string self_localization_pose_topic_name;
  std::string motion_reference_path_topic_name;
  std::string motion_reference_speed_topic_name;
  std::string motion_reference_pose_topic_name;
  std::string motion_reference_remaining_path_topic_name;

  //Functions
  void correctPath();
  float max(float d1,float d2);
  float min(float d1,float d2);
  void speedMovingStraight();
  void getOutput();  
  void speedMovingAround();  
  void nextState();
  void calculateTransitionToTarget();
  void setNextReferencePoint();
  void endingMovement();
  void publishValues(float dx, float dy, float dz);  
  void publishRemainingPoints();
  //Callback functions
  void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void motionReferencePathCallback(const nav_msgs::Path::ConstPtr& msg);
};
#endif 

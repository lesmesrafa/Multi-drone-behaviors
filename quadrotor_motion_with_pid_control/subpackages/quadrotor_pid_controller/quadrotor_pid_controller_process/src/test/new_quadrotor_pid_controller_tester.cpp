/*!*******************************************************************************************
 *  \file       new_quadrotor_pid_controller_tester.cpp
 *  \brief      Flight Motion PID Controller tester.
 *  \details    This code tests different flight modes and write data in csv files using NEW topics.
 *              PARAMETERS:
 *                -FREQUENCY: The frequency of the test in Herz
 *                -PATH: Location where the csv is going to be saved
 *                -MODE: The possible values are TRAJECTORY, SPEED, POSE.
 *                -DRONE: drone configuration (drone1, drone2, ...)
 *                -ITERATIONS: Number of iterations
 *              
 *  NOTE:       The variable init_control_mode located in the quadrotor_controller_config file 
 *              must be changed to "speed" or "position" in order to work properly.
 *                
 *  \authors    Alberto Rodelgo Perales
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

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "droneMsgsROS/droneTrajectoryControllerControlMode.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneCommand.h"
#include "droneMsgsROS/droneStatus.h"
#include <droneMsgsROS/InitiateBehaviors.h>
#include <aerostack_msgs/BehaviorSrv.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include <nav_msgs/Path.h>
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/MotionControlMode.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "aerostack_msgs/ControlMode.h"

//CONSTANTS: 
const int FREQUENCY = 20;                         //LOOP RATE
const std::string PATH = "/Documentos/Pruebas/";  //PATH WHERE CSV DATA IS GOING TO BE STORED
const std::string MODE = "POSE";                  //OPTIONS: SPEED; POSE; TRAJECTORY
const std::string DRONE = "drone1";               //Drone configuration
const int ITERATIONS = 700;                       //Loop iterations (it changes the amount of data stored in CSV files)

//CSV FILES
std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;
std::ofstream myfile4;
std::ofstream myfile5;

int droneStatusAux;

  //Callback self localization Pose  
  void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    myfile1 << msg->pose.position.x <<","<< msg->pose.position.y <<","<< msg->pose.position.z<<"\n";  //write data to csv file
  }
  //Callback self localization Speed
  void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    myfile2 << msg->twist.linear.x <<","<< msg->twist.linear.y <<","<< msg->twist.linear.z<<","<< msg->twist.angular.z<<","<< msg->twist.angular.y <<","<< msg->twist.angular.x<<"\n"; //write data to csv file
  }

  //Callback reference pose
  void referencePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg){
    myfile3 << msg->x <<","<< msg->y <<","<< msg->z<<","<< msg->yaw<<","<< msg->pitch <<","<< msg->roll<<"\n"; //write data to csv file
  }
  //Callback Assumed Ground Speed
  void assumedGroundSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    myfile4 << msg->twist.linear.x <<","<< msg->twist.linear.y <<","<< msg->twist.linear.z<<"\n"; //write data to csv file
  }
  //Callback actuator command multirotor command
  void actuatorCommandMultirotorCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg){
    myfile5 << msg->yaw_rate<<","<< msg->pitch <<","<< msg->roll<<"\n"; //write data to csv file
  }

  //Callback reference status
  void referenceStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg){
    droneStatusAux = msg->status;
  }

//Set control mode
bool setControlMode(ros::ServiceClient setControlModeClientSrv,int new_control_mode);

int main(int argc,char **argv) {
    try {
      ROS_INFO("Creating files...");
      std::string path(getenv("HOME"));
      path += PATH;
      //It stores the data in 5 different files
      myfile1.open(path+"selfPose.csv");
      myfile2.open(path+"selfSpeed.csv");
      myfile3.open(path+"RefPose.csv");
      myfile4.open(path+"AssumedGSpeed.csv");
      myfile5.open(path+"ActuatorCommand.csv");
      //Set precision to 5 decimals
      myfile1 << std::setprecision(5) << std::fixed;
      myfile2 << std::setprecision(5) << std::fixed;
      myfile3 << std::setprecision(5) << std::fixed;
      myfile4 << std::setprecision(5) << std::fixed;
      myfile5 << std::setprecision(5) << std::fixed;
      myfile1 << "x,y,z,\n";   
      myfile2 << "dx,dy,dz,dyaw,dpitch,droll,\n";
      myfile3 << "x,y,z,yaw,pitch,roll,\n";
      myfile4 << "dx,dy,dz,\n";
      myfile5 << "yaw_rate, pitch, roll,\n";

      //Ros Init
      ros::init(argc, argv, "testing_trajPlanPub");
      ros::NodeHandle n;
      ros::NodeHandle nspeed;
      ros::NodeHandle nPose;
      ros::NodeHandle nMode;
      ros::NodeHandle nCommand;

      //Publishers
      ros::Publisher refPubl = n.advertise<nav_msgs::Path>("/"+DRONE+"/motion_reference/trajectory", 1, true);
      ros::Publisher speedPub = nspeed.advertise<geometry_msgs::TwistStamped>("/"+DRONE+"/motion_reference/ground_speed", 1, true);
      ros::Publisher posePub = nPose.advertise<geometry_msgs::PoseStamped>("/"+DRONE+"/motion_reference/pose", 1, true);
      ros::Publisher commandPub = nCommand.advertise<droneMsgsROS::droneCommand>("/"+DRONE+"/command/high_level", 1, true);

      //Subscribers
      ros::Subscriber sub = n.subscribe("/"+DRONE+"/self_localization/pose", 1, selfLocalizationPoseCallback);
      ros::Subscriber sub2 = n.subscribe("/"+DRONE+"/self_localization/speed", 1, selfLocalizationSpeedCallback);
      ros::Subscriber sub3 = n.subscribe("/"+DRONE+"/trajectoryControllerPositionReferencesRebroadcast", 1, referencePoseCallback); 
      ros::Subscriber sub4 = n.subscribe("/"+DRONE+"/motion_reference/assumed_ground_speed", 1, assumedGroundSpeedCallback);
      ros::Subscriber sub5 = n.subscribe("/"+DRONE+"/actuator_command/multirotor_command", 1, actuatorCommandMultirotorCallback); 
      ros::Subscriber statusSub = n.subscribe("/"+DRONE+"/status", 1, referenceStatusCallback);
      
      //Services
      ros::ServiceClient startTrajectoryControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+DRONE+"/quadrotor_pid_controller_process/start");
      ros::ServiceClient setControlModeClientSrv = n.serviceClient<aerostack_msgs::SetControlMode>("/" + DRONE + "/set_control_mode");
      std_srvs::Empty req;
      startTrajectoryControllerClientSrv.call(req);
      std::string initiate_behaviors;
      std::string activate_behavior;
      activate_behavior="activate_behavior";
      droneMsgsROS::InitiateBehaviors msg;
      aerostack_msgs::BehaviorSrv::Request msg2;
      aerostack_msgs::BehaviorSrv::Response res;
      aerostack_msgs::BehaviorCommand behavior;
      behavior.name = "SELF_LOCALIZE_BY_ODOMETRY";
      msg2.behavior = behavior;
      n.param<std::string>("initiate_behaviors", initiate_behaviors, "initiate_behaviors");
      ros::ServiceClient initiate_behaviors_srv;
      ros::ServiceClient activate_behavior_srv;
      initiate_behaviors_srv=n.serviceClient<droneMsgsROS::InitiateBehaviors>("/"+DRONE+"/"+initiate_behaviors);
      activate_behavior_srv=n.serviceClient<aerostack_msgs::BehaviorSrv>("/"+DRONE+"/"+activate_behavior);

      initiate_behaviors_srv.call(msg);
      activate_behavior_srv.call(msg2,res);

      //Rate
      ros::Rate loop_rate(FREQUENCY);
      bool aux = true;
      int count;

    //Starting processes properly (sleep 3 seconds before testing)
    sleep(3);

    droneMsgsROS::droneTrajectoryControllerControlMode mode;
    std::cout<<"MODE: "<< MODE << std::endl;
    
    // LOOP
    for(count = 0; ros::ok() && count < ITERATIONS;count++){

      //Mode trajectory: it sends a trajectory to the quadrotor controller
      // if (MODE  == "TRAJECTORY"){
      //   geometry_msgs::PoseStamped trajectory_poses;
      //   nav_msgs::Path refMsg;     
      
      //   if (aux){
      //     // Trajectory points:
      //     trajectory_poses.pose.position.x=0.0;
      //     trajectory_poses.pose.position.y=0.0;
      //     trajectory_poses.pose.position.z=1.0;
      //     trajectory_poses.pose.orientation.w = -0.781;
      //     trajectory_poses.pose.orientation.x =  0.002;
      //     trajectory_poses.pose.orientation.y =  0.592;
      //     trajectory_poses.pose.orientation.z = -0.198;
      //     //refMsg.poses.push_back(trajectory_poses);
      //     trajectory_poses.pose.position.x=2.0;
      //     trajectory_poses.pose.position.y=2.0;
      //     trajectory_poses.pose.position.z=1.0;
      //     trajectory_poses.pose.orientation.w = -0.781;
      //     trajectory_poses.pose.orientation.x =  0.002;
      //     trajectory_poses.pose.orientation.y =  0.592;
      //     trajectory_poses.pose.orientation.z = -0.198;
      //     refMsg.poses.push_back(trajectory_poses);
      //     trajectory_poses.pose.position.x=5.0;
      //     trajectory_poses.pose.position.y=2.0;
      //     trajectory_poses.pose.position.z=5.0;
      //     trajectory_poses.pose.orientation.w =  -0.999;
      //     trajectory_poses.pose.orientation.x =   0.001;
      //     trajectory_poses.pose.orientation.y =  -0.040;
      //     trajectory_poses.pose.orientation.z =  -0.004;         
      //     refMsg.poses.push_back(trajectory_poses);
      //     trajectory_poses.pose.position.x=5.0;
      //     trajectory_poses.pose.position.y=-1.0;
      //     trajectory_poses.pose.position.z=5.0;
      //     trajectory_poses.pose.orientation.w =   0.703;
      //     trajectory_poses.pose.orientation.x =  -0.010;
      //     trajectory_poses.pose.orientation.y =   0.678;
      //     trajectory_poses.pose.orientation.z =  -0.214;      
      //     refMsg.poses.push_back(trajectory_poses);
      //     trajectory_poses.pose.position.x=4.0;
      //     trajectory_poses.pose.position.y=1.0;
      //     trajectory_poses.pose.position.z=5.0;
      //     trajectory_poses.pose.orientation.w =   0.585;
      //     trajectory_poses.pose.orientation.x =   0.011;
      //     trajectory_poses.pose.orientation.y =  -0.766;
      //     trajectory_poses.pose.orientation.z =   0.265;      
      //     refMsg.poses.push_back(trajectory_poses);
          
      //     std::cout<<"Publishing trajectory..."<<std::endl;
      //     refPubl.publish(refMsg);
      //     std::cout<<"Trajectory published"<<std::endl;
      //     aux = false;
      //   }
      // }

      //Mode speed: it sends dx,dy,dz and dyaw to the quadrotor controller
      if (MODE  == "SPEED" && aux){
        geometry_msgs::TwistStamped speed;
        speed.twist.linear.x= 4;
        speed.twist.linear.y= 1;    
        speed.twist.linear.z= 0; 
        speed.twist.angular.x=  0.157;
        speed.twist.angular.y= -0.749;    
        speed.twist.angular.z=  0.364;     
        setControlMode(setControlModeClientSrv,aerostack_msgs::MotionControlMode::SPEED);
        speedPub.publish(speed);
	     aux = false;
      }

      //Mode pose: it sends x,y,z and yaw to the quadrotor controller
      if (MODE == "POSE" && aux){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=6;
        pose.pose.position.y=4;
        pose.pose.position.z=1;
        pose.pose.orientation.w= -0.841;
        pose.pose.orientation.x= -0.008;
        pose.pose.orientation.y=  0.509;
        pose.pose.orientation.z= -0.184;        
        setControlMode(setControlModeClientSrv,aerostack_msgs::MotionControlMode::POSE);  
        posePub.publish(pose);    
	      aux = false;        
      }

      //TakeOff (if the vehicle is landed)
      if (droneStatusAux == 2){
        std::cout<<"Taking Off..."<< std::endl;
        droneMsgsROS::droneCommand takeoffMSG;
        takeoffMSG.mode = 1;
        commandPub.publish(takeoffMSG);
      }

      //MOVE (if the vehicle is hovering)
      if (droneStatusAux == 4){
        std::cout<<"Moving..."<<std::endl;
        droneMsgsROS::droneCommand movingMSG;
        movingMSG.mode = 4;
        commandPub.publish(movingMSG);
      }

      ros::spinOnce();
      loop_rate.sleep();
  }
    ROS_INFO("Test finished.");  
    myfile1.close();
    myfile2.close();
    myfile3.close();
    myfile4.close();
    myfile5.close();
    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }

    return 1;
}

//Set control mode
bool setControlMode(ros::ServiceClient setControlModeClientSrv,int new_control_mode){
  // Prepare service message
  aerostack_msgs::SetControlMode setControlModeSrv;
  setControlModeSrv.request.controlMode.mode = new_control_mode;
  // use service
  if (setControlModeClientSrv.call(setControlModeSrv)){
    return setControlModeSrv.response.ack;
  }else{
    return false;
  }
}


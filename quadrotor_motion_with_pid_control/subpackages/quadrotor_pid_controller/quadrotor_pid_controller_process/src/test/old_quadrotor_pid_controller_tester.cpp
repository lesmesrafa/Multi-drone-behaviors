/*!*******************************************************************************************
 *  \file       old_quadrotor_pid_controller_tester.cpp
 *  \brief      Flight Motion PID Controller tester.
 *  \details    This code tests different flight modes and write data in csv files using OLD topics.
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
#include "droneMsgsROS/dronePositionRefCommand.h"
#include "droneMsgsROS/dronePositionTrajectoryRefCommand.h"
#include "droneMsgsROS/dronePositionRefCommandStamped.h"
#include "droneMsgsROS/droneTrajectoryControllerControlMode.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "droneMsgsROS/droneCommand.h"
#include "droneMsgsROS/droneStatus.h"
#include <droneMsgsROS/InitiateBehaviors.h>
#include <aerostack_msgs/BehaviorSrv.h>

//CONSTANTS: 
const int FREQUENCY = 20;                         //LOOP RATE
const std::string PATH = "/Documentos/Pruebas/";  //PATH WHERE CSV DATA IS GOING TO BE STORED
const std::string MODE = "SPEED";                 //OPTIONS: SPEED; POSE; TRAJECTORY
const std::string DRONE = "drone1";               //Drone configuration
const int ITERATIONS = 700;                       //Loop iterations (it changes the amount of data stored in CSV files)

//CSV FILES
std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;
std::ofstream myfile4;

int droneStatusAux;

  //Callback estimated_pose
  void estimatedPoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg){
   myfile1 << msg->x <<","<< msg->y <<","<< msg->z<<","<< msg->yaw<<","<< msg->pitch <<","<< msg->roll<<"\n"; //write data to csv file
  }
  //Callback estimated_speed
  void estimatedSpeedCallback(const droneMsgsROS::droneSpeeds::ConstPtr& msg){
    myfile2 << msg->dx <<","<< msg->dy <<","<< msg->dz<<","<< msg->dyaw<<","<< msg->dpitch <<","<< msg->droll<<"\n"; //write data to csv file
  }

  //Callback reference pose
  void referencePoseCallback(const droneMsgsROS::dronePose::ConstPtr& msg){
    myfile3 << msg->x <<","<< msg->y <<","<< msg->z<<","<< msg->yaw<<","<< msg->pitch <<","<< msg->roll<<"\n"; //write data to csv file
  }
  //Callback reference speed
  void referenceSpeedCallback(const droneMsgsROS::droneSpeeds::ConstPtr& msg){
    myfile4 << msg->dx <<","<< msg->dy <<","<< msg->dz<<"\n"; //write data to csv file
  }
  //Callback reference status
  void referenceStatusCallback(const droneMsgsROS::droneStatus::ConstPtr& msg){
    droneStatusAux = msg->status;
  }


int main(int argc,char **argv) {
    try {
      ROS_INFO("Creating files...");
      std::string path(getenv("HOME"));
      path += PATH;
      //It stores the data in 4 different files
      myfile1.open(path+"EstimatedPose.csv");
      myfile2.open(path+"EstimatedSpeed.csv");
      myfile3.open(path+"ReferencedPose.csv");
      myfile4.open(path+"ReferenceSpeed.csv"); 
      //Set precision to 5 decimals      
      myfile1 << std::setprecision(5) << std::fixed;
      myfile2 << std::setprecision(5) << std::fixed;
      myfile3 << std::setprecision(5) << std::fixed;
      myfile4 << std::setprecision(5) << std::fixed;
      myfile1 << "x,y,z,yaw,pitch,roll\n";   
      myfile2 << "dx,dy,dz,dyaw,dpitch,droll,\n";
      myfile3 << "x,y,z,yaw,pitch,roll,\n";
      myfile4 << "dx,dy,dz,\n";

      //Ros Init
      ros::init(argc, argv, "testing_trajPlanPub");
      ros::NodeHandle n;
      ros::NodeHandle nspeed;
      ros::NodeHandle nPose;
      ros::NodeHandle nMode;
      ros::NodeHandle nCommand;

      //Publishers
      ros::Publisher dronePositionTrajectoryRefCommandPubl = n.advertise<droneMsgsROS::dronePositionTrajectoryRefCommand>("/"+DRONE+"/droneTrajectoryAbsRefCommand", 1, true);
      ros::Publisher speedPub = nspeed.advertise<droneMsgsROS::droneSpeeds>("/"+DRONE+"/droneSpeedsRefs", 1, true);
      ros::Publisher posePub = nPose.advertise<droneMsgsROS::dronePositionRefCommandStamped>("/"+DRONE+"/dronePositionRefs", 1, true);
      ros::Publisher modePub = nMode.advertise<droneMsgsROS::droneTrajectoryControllerControlMode>("/"+DRONE+"/controlMode", 1, true);
      ros::Publisher commandPub = nCommand.advertise<droneMsgsROS::droneCommand>("/"+DRONE+"/command/high_level", 1, true);

      //Subscribers
      ros::Subscriber sub = n.subscribe("/"+DRONE+"/estimated_pose", 1, estimatedPoseCallback);
      ros::Subscriber sub2 = n.subscribe("/"+DRONE+"/estimated_speed", 1, estimatedSpeedCallback);
      ros::Subscriber sub3 = n.subscribe("/"+DRONE+"/trajectoryControllerPositionReferencesRebroadcast", 1, referencePoseCallback);
      ros::Subscriber sub4 = n.subscribe("/"+DRONE+"/trajectoryControllerSpeedReferencesRebroadcast", 1, referenceSpeedCallback);
      ros::Subscriber statusSub = n.subscribe("/"+DRONE+"/status", 1, referenceStatusCallback);
      
      //Services
      ros::ServiceClient startTrajectoryControllerClientSrv=n.serviceClient<std_srvs::Empty>("/"+DRONE+"/quadrotor_pid_controller_process/start");
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
      if (MODE  == "TRAJECTORY"){
        droneMsgsROS::dronePositionRefCommand dronePositionRefCommandAux;
        droneMsgsROS::dronePositionTrajectoryRefCommand dronePositionTrajectoryRefCommandMsg;
        dronePositionTrajectoryRefCommandMsg.is_periodic=true;
        dronePositionTrajectoryRefCommandMsg.initial_checkpoint=1;      

        if (aux){
          // Trajectory points:
          dronePositionRefCommandAux.x=0.0;
          dronePositionRefCommandAux.y=0.0;
          dronePositionRefCommandAux.z=1.0;
          dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
          dronePositionRefCommandAux.x=4.0;
          dronePositionRefCommandAux.y=7.0;
          dronePositionRefCommandAux.z=1.0;
          dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
          dronePositionRefCommandAux.x=7.0;
          dronePositionRefCommandAux.y=7.0;
          dronePositionRefCommandAux.z=5.0;
          dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
          dronePositionRefCommandAux.x=7.0;
          dronePositionRefCommandAux.y=4.0;
          dronePositionRefCommandAux.z=5.0;
          dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
          dronePositionRefCommandAux.x=6.0;
          dronePositionRefCommandAux.y=6.0;
          dronePositionRefCommandAux.z=5.0;
          dronePositionTrajectoryRefCommandMsg.droneTrajectory.push_back(dronePositionRefCommandAux);
          
          std::cout<<"Publishing trajectory....."<<std::endl;
          dronePositionTrajectoryRefCommandPubl.publish(dronePositionTrajectoryRefCommandMsg);
          std::cout<<"Trajectory published"<<std::endl;
          aux = false;
        }
      }
      //Mode speed: it sends dx and dy to the quadrotor controller
      if (MODE  == "SPEED"){
        mode.mode = 3;
        modePub.publish(mode);
        droneMsgsROS::droneSpeeds speed;
        speed.dx=4;
        speed.dy=1;
        speed.dz=0;   
        speedPub.publish(speed);          
      }

      //Mode pose: it sends x,y and z to the quadrotor controller
      if (MODE == "POSE"){
        mode.mode = 2;
        modePub.publish(mode);        
        droneMsgsROS::dronePositionRefCommandStamped pose;
        pose.position_command.x=6;
        pose.position_command.y=4;
        pose.position_command.z=1;   
        posePub.publish(pose);    
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
      aux = false;
  }
    ROS_INFO("Test finished.");  
    myfile1.close();
    myfile2.close();
    myfile3.close();
    myfile4.close();
    } catch (std::exception &ex) {
        std::cout << "[ROSNODE] Exception :" << ex.what() << std::endl;
    }
    return 1;
}
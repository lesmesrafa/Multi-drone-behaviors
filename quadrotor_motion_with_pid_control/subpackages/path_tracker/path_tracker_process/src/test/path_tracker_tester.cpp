/*!*******************************************************************************************
 *  \file       path_tracker_tester.cpp
 *  \brief      Path Tracker tester implementation file.
 *  \details    This file publish different paths to the path tracker
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
#include "aerostack_msgs/FlightActionCommand.h"
#include "aerostack_msgs/FlightState.h"
#include <nav_msgs/Path.h>
#include "aerostack_msgs/ControlMode.h"
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/MotionControlMode.h"
#include "math.h"

//CONSTANTS: 
const int FREQUENCY = 20;                         //LOOP RATE
const int TEST = 3;                               
const std::string DRONE = "drone111";               //Drone configuration


int droneStatusAux;

//Callback reference status
void referenceStatusCallback(const aerostack_msgs::FlightState::ConstPtr& msg){
  droneStatusAux = msg->state;
}

//Set control mode
bool setControlMode(ros::ServiceClient setControlModeClientSrv,int new_control_mode);

int main(int argc,char **argv) {
    try {

      //Ros Init
      ros::init(argc, argv, "testing_path_tracker");
      ros::NodeHandle n;
      ros::NodeHandle nCommand;

      //Publishers
      ros::Publisher refPubl = n.advertise<nav_msgs::Path>("/"+DRONE+"/motion_reference/path", 1, true);
      ros::Publisher commandPub = nCommand.advertise<aerostack_msgs::FlightActionCommand>("/"+DRONE+"/actuator_command/flight_action", 1, true);

      //Subscriber 
      ros::Subscriber statusSub = n.subscribe("/"+DRONE+"/self_localization/flight_state", 1, referenceStatusCallback);
      
      //Services
      ros::ServiceClient startPathTrackerSrv=n.serviceClient<std_srvs::Empty>("/"+DRONE+"/path_tracker_process/start");
      ros::ServiceClient setControlModeClientSrv = n.serviceClient<aerostack_msgs::SetControlMode>("/" + DRONE + "/set_control_mode");
      std_srvs::Empty req;
      startPathTrackerSrv.call(req);

      //Variables
      ros::Rate loop_rate(FREQUENCY);
      bool aux = true;
      geometry_msgs::PoseStamped trajectory_poses;
      nav_msgs::Path refMsg;  

    //Starting processes properly (sleep 3 seconds before testing)
    sleep(3);
    setControlMode(setControlModeClientSrv,aerostack_msgs::MotionControlMode::SPEED_3D);

    std::cout<<"TEST: "<< TEST << std::endl;
    
    // LOOP
    while(ros::ok()){
      if (aux){
        switch(TEST){
          case 1: //Random trajectory 
          {
          float random;
          srand (time(NULL)); //Random seed
          // Trajectory points:
            trajectory_poses.pose.position.x=2.0;
            trajectory_poses.pose.position.y=2.0;
            trajectory_poses.pose.position.z=1.0;
            refMsg.poses.push_back(trajectory_poses);           
            for (int i=0; i<200; i++){
              random = (rand() % 10); // generate random number between 0 and 10
              random = random/10;
              trajectory_poses.pose.position.x+=random;
              if (i % 2 == 0) random = random * (-1);
              trajectory_poses.pose.position.y+=random;
              trajectory_poses.pose.position.z=1.0;
              refMsg.poses.push_back(trajectory_poses);            
            }
            std::cout<<"Publishing trajectory..."<<std::endl;
            refPubl.publish(refMsg);
            std::cout<<"Trajectory published"<<std::endl;
            aux = false;
          }
          break;

          case 2: //Specific path with orientation
          {
            trajectory_poses.pose.position.x=2.0;
            trajectory_poses.pose.position.y=2.0;
            trajectory_poses.pose.position.z=1.0;
            trajectory_poses.pose.orientation.w = -0.781;
            trajectory_poses.pose.orientation.x =  0.002;
            trajectory_poses.pose.orientation.y =  0.592;
            trajectory_poses.pose.orientation.z = -0.198;
            refMsg.poses.push_back(trajectory_poses);
            trajectory_poses.pose.position.x=5;
            trajectory_poses.pose.position.y=2.0;
            trajectory_poses.pose.position.z=5.0;
            trajectory_poses.pose.orientation.w =  -0.999;
            trajectory_poses.pose.orientation.x =   0.001;
            trajectory_poses.pose.orientation.y =  -0.040;
            trajectory_poses.pose.orientation.z =  -0.004;         
            refMsg.poses.push_back(trajectory_poses);
            trajectory_poses.pose.position.x=2.3;
            trajectory_poses.pose.position.y=1.7;
            trajectory_poses.pose.position.z=3.0;
            trajectory_poses.pose.orientation.w =   0.703;
            trajectory_poses.pose.orientation.x =  -0.010;
            trajectory_poses.pose.orientation.y =   0.678;
            trajectory_poses.pose.orientation.z =  -0.214;      
            refMsg.poses.push_back(trajectory_poses);
            trajectory_poses.pose.position.x=4.0;
            trajectory_poses.pose.position.y=1.0;
            trajectory_poses.pose.position.z=1.0;
            trajectory_poses.pose.orientation.w =   0.585;
            trajectory_poses.pose.orientation.x =   0.011;
            trajectory_poses.pose.orientation.y =  -0.766;
            trajectory_poses.pose.orientation.z =   0.265;      
            refMsg.poses.push_back(trajectory_poses);
            
            std::cout<<"Publishing trajectory..."<<std::endl;
            refPubl.publish(refMsg);
            std::cout<<"Trajectory published"<<std::endl;
            aux = false;    
          }      
          break;

          case 3: //Path with small distances and curves (cos(x))
          {
              for (int j = 0; j<100; j++){         
                for (int i=0; i<10; i++){
                  trajectory_poses.pose.position.x+=0.1;
                  trajectory_poses.pose.position.y=cos(trajectory_poses.pose.position.x);
                  trajectory_poses.pose.position.z=1.0;
                  refMsg.poses.push_back(trajectory_poses);            
                }
            }                                                                  
            std::cout<<"Publishing trajectory..."<<std::endl;
            refPubl.publish(refMsg);
            std::cout<<"Trajectory published"<<std::endl;
            aux = false;
          }          
          break;
          case 4: //First point is self_location and other targets are same x,y but different z
          {
            trajectory_poses.pose.position.x=0.0;
            trajectory_poses.pose.position.y=0.0;
            trajectory_poses.pose.position.z=1.0;
            refMsg.poses.push_back(trajectory_poses);
            trajectory_poses.pose.position.x=2.0;
            trajectory_poses.pose.position.y=1.0;
            trajectory_poses.pose.position.z=1.0;
            refMsg.poses.push_back(trajectory_poses);
            trajectory_poses.pose.position.x=2.0;
            trajectory_poses.pose.position.y=1.0;
            trajectory_poses.pose.position.z=2.0;
            refMsg.poses.push_back(trajectory_poses);          
            trajectory_poses.pose.position.x=3.0;
            trajectory_poses.pose.position.y=1.0;
            trajectory_poses.pose.position.z=2.0;
            refMsg.poses.push_back(trajectory_poses);  
            std::cout<<"Publishing trajectory..."<<std::endl;
            refPubl.publish(refMsg);
            std::cout<<"Trajectory published"<<std::endl;
            aux = false;            
          }
          break;
          default:
          break;
        }        
      }

      //TakeOff (if the vehicle is landed)
      if (droneStatusAux == aerostack_msgs::FlightState::LANDED){
        std::cout<<"Taking Off..."<< std::endl;
        aerostack_msgs::FlightActionCommand takeoffMSG;
        takeoffMSG.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
        commandPub.publish(takeoffMSG);
      }

      //MOVE (if the vehicle is hovering)
      if (droneStatusAux == aerostack_msgs::FlightState::HOVERING){
        std::cout<<"Moving..."<<std::endl;
        aerostack_msgs::FlightActionCommand movingMSG;
        movingMSG.action = aerostack_msgs::FlightActionCommand::MOVE;
        commandPub.publish(movingMSG);
      }

      ros::spinOnce();
      loop_rate.sleep();
  }
    ROS_INFO("Test finished.");  
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
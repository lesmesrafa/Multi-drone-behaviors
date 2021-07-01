/*!********************************************************************************
 * \brief     keep_hovering implementation
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

#include "../include/behavior_keep_hovering_with_pid_control.h"
#include <iostream>
#include <fstream>

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorKeepHoveringWithPidControl behavior;
  behavior.start();
  return 0;
}

BehaviorKeepHoveringWithPidControl::BehaviorKeepHoveringWithPidControl() : BehaviorExecutionManager() { 
  setName("keep_hovering_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorKeepHoveringWithPidControl::~BehaviorKeepHoveringWithPidControl() {}

void BehaviorKeepHoveringWithPidControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace(); 
 
  ros::param::get("~estimated_speed_topic", self_localization_speed_str);
  ros::param::get("~estimated_pose_topic", self_localization_pose_str);
  ros::param::get("~controllers_topic", command_high_level_str);
  ros::param::get("~motion_reference_speed_topic", motion_reference_speed_str);
  ros::param::get("~motion_reference_pose_topic", motion_reference_pose_str);
  ros::param::get("~set_control_mode_service_name", set_control_mode_srv);
  ros::param::get("~status_topic", status_str);

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorKeepHoveringWithPidControl::statusCallBack, this);
}

bool BehaviorKeepHoveringWithPidControl::checkSituation()
{
  //Quadrotor is FLYING
  if ((status_msg.state != aerostack_msgs::FlightState::LANDED) && 
      (status_msg.state != aerostack_msgs::FlightState::UNKNOWN)&&
      (status_msg.state != aerostack_msgs::FlightState::LANDING)){
    return true;
  }
  else{
    setErrorMessage("Error: Drone is not flying");
    return false;
  }
}

void BehaviorKeepHoveringWithPidControl::onActivate()
{
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  
  motion_reference_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/"+motion_reference_pose_str, 1,true);
  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);

  received_speed = false;
  aerostack_msgs::FlightActionCommand high_level_command;
  geometry_msgs::TwistStamped motion_reference_speed;
  std_msgs::Header header;

  motion_reference_speed.twist.linear.x = 0.0;
  motion_reference_speed.twist.linear.y = 0.0;
  motion_reference_speed.twist.linear.z = 0.0;
  motion_reference_speed.twist.angular.x = 0.0;
  motion_reference_speed.twist.angular.y = 0.0;
  motion_reference_speed.twist.angular.z = 0.0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  header.frame_id = "behavior_keep_hovering_with_pid_control";  
  high_level_command.header = header;
  high_level_command.action = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(high_level_command);  
}

void BehaviorKeepHoveringWithPidControl::onDeactivate()
{

  /*std_msgs::Header header;
  header.frame_id = "behavior_keep_hovering_with_pid_control";*/

  /*aerostack_msgs::FlightActionCommand msg;
  msg.header = header;
  msg.mode = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(msg); */
  newPose = false;
  setControlModeClientSrv.shutdown();
  self_localization_pose_sub.shutdown();
  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  motion_reference_pose_pub.shutdown();
}

void BehaviorKeepHoveringWithPidControl::onExecute(){
  if (received_speed){
    if(!newPose){
      reference_pose  = estimated_pose_msg;
      newPose = true;
    }
    setControlMode(aerostack_msgs::MotionControlMode::POSE);
    motion_reference_pose_pub.publish(reference_pose);
  }
}

void BehaviorKeepHoveringWithPidControl::checkGoal(){}


void BehaviorKeepHoveringWithPidControl::checkProgress() {
  if (checkQuadrotorStopped()){
    distance = sqrt(pow(estimated_pose_msg.pose.position.x-reference_pose.pose.position.x,2)+
                    pow(estimated_pose_msg.pose.position.y-reference_pose.pose.position.y,2)+
                    pow(estimated_pose_msg.pose.position.z-reference_pose.pose.position.z,2));

    if (distance > 1) 
      BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}


void BehaviorKeepHoveringWithPidControl::checkProcesses() 
{ 
 
}

bool BehaviorKeepHoveringWithPidControl::checkQuadrotorStopped()
{

  if (abs(estimated_speed_msg.twist.linear.x) <= 0.1 && abs(estimated_speed_msg.twist.linear.y) <= 0.1 && abs(estimated_speed_msg.twist.linear.z) <= 0.1 &&
      abs(estimated_speed_msg.twist.angular.x) <= 0.1 && abs(estimated_speed_msg.twist.angular.y) <= 0.1 && abs(estimated_speed_msg.twist.angular.z) <= 0.1 ){
      return true;
  }else{
    return false;
  }
}

//Set control mode
bool BehaviorKeepHoveringWithPidControl::setControlMode(int new_control_mode){
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

void BehaviorKeepHoveringWithPidControl::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){
  estimated_speed_msg = msg; received_speed = true;
}
void BehaviorKeepHoveringWithPidControl::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}
void BehaviorKeepHoveringWithPidControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

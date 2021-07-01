/*!********************************************************************************
 * \brief     keep_moving implementation
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

#include "../include/behavior_keep_moving_with_pid_control.h"
#include <iostream>
#include <fstream>

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorKeepMovingWithPidControl behavior;
  behavior.start();
  return 0;
}

BehaviorKeepMovingWithPidControl::BehaviorKeepMovingWithPidControl() : BehaviorExecutionManager() { 
  setName("keep_moving_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorKeepMovingWithPidControl::~BehaviorKeepMovingWithPidControl() {}

void BehaviorKeepMovingWithPidControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();  

  ros::param::get("~estimated_speed_topic", self_localization_speed_str);
  ros::param::get("~controllers_topic", command_high_level_str);
  ros::param::get("~motion_reference_speed_topic", motion_reference_speed_str);
  ros::param::get("~set_control_mode_service_name", set_control_mode_srv);
  ros::param::get("~status_topic", status_str);

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorKeepMovingWithPidControl::statusCallBack, this);
  maximum_observed_y=0;
  minimum_observed_y=0;
  maximum_observed_x=0;
  minimum_observed_x=0;
}

bool BehaviorKeepMovingWithPidControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state != aerostack_msgs::FlightState::LANDED){
    return true;
  }else{
    setErrorMessage("Error: Drone is landed");
    return false;
  }
}

void BehaviorKeepMovingWithPidControl::checkGoal(){}


void BehaviorKeepMovingWithPidControl::checkProgress() {
    ros::Duration diff = ros::Time::now() - activation_time;
    if (diff.toSec() > 2){
      if(received_speed && direction == "LEFT")
    {
          if (!(abs(estimated_speed_msg.twist.linear.x)>=maximum_observed_x-0.01 && abs(estimated_speed_msg.twist.linear.y)<0.1)) 
          BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
    }
     else if(received_speed && direction == "RIGHT")
    {
       if (!(abs(estimated_speed_msg.twist.linear.x)>=minimum_observed_x-0.01 && abs(estimated_speed_msg.twist.linear.y)<0.1)) 
          BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
    }
     else if(received_speed && direction == "FORWARD")
    {
        if (!(abs(estimated_speed_msg.twist.linear.x)<0.1 && abs(estimated_speed_msg.twist.linear.y)>=maximum_observed_y-0.01)) 
           {BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
          }

    }
     else if(received_speed && direction == "BACKWARD")
    {
      if (!(abs(estimated_speed_msg.twist.linear.x)<0.1 && abs(estimated_speed_msg.twist.linear.y)>=minimum_observed_y-0.01)) 
          BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

    }
     int newy=estimated_speed_msg.twist.linear.y;
     int newx=estimated_speed_msg.twist.linear.x;
     maximum_observed_y=std::max(maximum_observed_y,newy);
     minimum_observed_y=std::min(minimum_observed_y,newy);
     maximum_observed_x=std::max(maximum_observed_x,newx);
     minimum_observed_x=std::min(minimum_observed_x,newx);
    }
    
}

void BehaviorKeepMovingWithPidControl::checkProcesses() 
{ 
 
}

void BehaviorKeepMovingWithPidControl::onExecute() 
{ 
 
}

//Set control mode
bool BehaviorKeepMovingWithPidControl::setControlMode(int new_control_mode){
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

void BehaviorKeepMovingWithPidControl::onActivate()
{

  maximum_observed_y=0;
  minimum_observed_y=0;
  maximum_observed_x=0;
  minimum_observed_x=0;
  estimated_speed_msg = *ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/" + nspace + "/"+self_localization_speed_str, node_handle, ros::Duration(1));

  //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorKeepMovingWithPidControl::selfLocalizationSpeedCallBack, this);
  //Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  
  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);
  //ros::Duration(0.5).sleep();//last value 4
  received_speed = false;
  std_msgs::Header header;

  //Get arguments
  std::string arguments=getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  direction = config_file["direction"].as<std::string>();
  double speed = config_file["speed"].as<double>();  
    if(direction == "LEFT")
    {
      motion_reference_speed.twist.linear.x=-speed;
      motion_reference_speed.twist.linear.y=0;
      motion_reference_speed.twist.linear.z=0;

    }
    else if(direction == "RIGHT")
    {

      motion_reference_speed.twist.linear.x=speed;
      motion_reference_speed.twist.linear.y=0;
      motion_reference_speed.twist.linear.z=0;

    }
    else if(direction == "FORWARD")
    {

      motion_reference_speed.twist.linear.x=0;
      motion_reference_speed.twist.linear.y=speed;
      motion_reference_speed.twist.linear.z=0;
    }
    else if(direction == "BACKWARD")
    {

      motion_reference_speed.twist.linear.x=0;
      motion_reference_speed.twist.linear.y=-speed;
      motion_reference_speed.twist.linear.z=0;
    }

    setControlMode(aerostack_msgs::MotionControlMode::SPEED);
    motion_reference_speed_pub.publish(motion_reference_speed);
    header.frame_id = "behavior_keep_moving_with_pid_control";  
    high_level_command.header = header;
    high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
    command_high_level_pub.publish(high_level_command);
    activation_time = ros::Time::now();

}

void BehaviorKeepMovingWithPidControl::onDeactivate()
{
  std_msgs::Header header;
  header.frame_id = "behavior_keep_moving_with_pid_control";
  aerostack_msgs::FlightActionCommand msg;
  msg.header = header;
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  motion_reference_speed.twist.linear.x=0;
  motion_reference_speed.twist.linear.y=0;
  motion_reference_speed.twist.linear.z=0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  command_high_level_pub.publish(msg);

  setControlModeClientSrv.shutdown();
  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  motion_reference_speed_pub.shutdown();
}
void BehaviorKeepMovingWithPidControl::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){
  estimated_speed_msg = msg; received_speed = true;
}
void BehaviorKeepMovingWithPidControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

/*!********************************************************************************
 * \brief     rotate implementation
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

#include "../include/behavior_rotate_with_pid_control.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorRotateWithPidControl behavior;
  behavior.start();
  return 0;
}

BehaviorRotateWithPidControl::BehaviorRotateWithPidControl() : BehaviorExecutionManager(){ 
  setName("rotate_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorRotateWithPidControl::~BehaviorRotateWithPidControl() {}

void BehaviorRotateWithPidControl::onConfigure(){
  node_handle = getNodeHandle();
  nspace = getNamespace();

  node_handle.param<std::string>("estimated_speed_topic", self_localization_speed_str,"self_localization/speed");
  node_handle.param<std::string>("estimated_pose_topic", self_localization_pose_str,"self_localization/pose");
  node_handle.param<std::string>("controllers_topic", command_high_level_str,"actuator_command/flight_action");
  node_handle.param<std::string>("motion_reference_speed_topic", motion_reference_speed_str,"motion_reference/speed");
  node_handle.param<std::string>("motion_reference_pose_topic", motion_reference_pose_str,"motion_reference/pose");
  node_handle.param<std::string>("set_control_mode_service_name", set_control_mode_srv,"set_control_mode");
  node_handle.param<std::string>("status_topic", status_str,"self_localization/flight_state");
  
  //Subscribers
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_pose_str, 1, &BehaviorRotateWithPidControl::selfLocalizationPoseCallBack, this);
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorRotateWithPidControl::selfLocalizationSpeedCallBack, this);
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorRotateWithPidControl::statusCallBack, this);
}

bool BehaviorRotateWithPidControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    setErrorMessage("Error: Drone is landed");
    return false;
  }
return true;
}

void BehaviorRotateWithPidControl::checkGoal(){ 
  if (!quadrotor_moving){
      current_angle = 0;
      angle2 = 0;
      if(!(estimated_pose_msg.pose.orientation.w == 0 && estimated_pose_msg.pose.orientation.x == 0 && estimated_pose_msg.pose.orientation.y == 0 && estimated_pose_msg.pose.orientation.z == 0)){
          current_angle = atan2(2.0 * (estimated_pose_msg.pose.orientation.z * estimated_pose_msg.pose.orientation.w + estimated_pose_msg.pose.orientation.x * estimated_pose_msg.pose.orientation.y) , 
                              - 1.0 + 2.0 * (estimated_pose_msg.pose.orientation.w * estimated_pose_msg.pose.orientation.w + estimated_pose_msg.pose.orientation.x * estimated_pose_msg.pose.orientation.x));    
          angle2 = atan2(2.0 * (reference_pose.pose.orientation.z * reference_pose.pose.orientation.w + reference_pose.pose.orientation.x * reference_pose.pose.orientation.y) , 
                              - 1.0 + 2.0 * (reference_pose.pose.orientation.w * reference_pose.pose.orientation.w + reference_pose.pose.orientation.x * reference_pose.pose.orientation.x));    
      }
      if (abs(abs(angle2) - abs(current_angle)) < 0.01 && abs(estimated_speed_msg.twist.angular.x) <= 0.1) BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
}

void BehaviorRotateWithPidControl::checkProgress(){
  if (!quadrotor_moving){
    distance = sqrt(pow(estimated_pose_msg.pose.position.x-reference_pose.pose.position.x,2)+
                    pow(estimated_pose_msg.pose.position.y-reference_pose.pose.position.y,2)+
                    pow(estimated_pose_msg.pose.position.z-reference_pose.pose.position.z,2));

    if (distance > 1){
      BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
    }
  }
}

//Set control mode
bool BehaviorRotateWithPidControl::setControlMode(int new_control_mode){
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

void BehaviorRotateWithPidControl::checkProcesses(){ 
 
}


void BehaviorRotateWithPidControl::onActivate(){
  //Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  
  motion_reference_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/"+motion_reference_pose_str, 1,true);
  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);
  
  //Subscribers
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_pose_str, 1, &BehaviorRotateWithPidControl::selfLocalizationPoseCallBack, this);
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorRotateWithPidControl::selfLocalizationSpeedCallBack, this);
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorRotateWithPidControl::statusCallBack, this);

  quadrotor_moving = true;
  geometry_msgs::TwistStamped motion_reference_speed;
  std_msgs::Header header;
  motion_reference_speed.twist.linear.x = 0.0;
  motion_reference_speed.twist.linear.y = 0.0;
  motion_reference_speed.twist.linear.z = 0.0;
  motion_reference_speed.twist.angular.x = 0.0;
  motion_reference_speed.twist.angular.y = 0.0;
  motion_reference_speed.twist.angular.z = 0.0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  header.frame_id = "behavior_rotate_with_pid_control";  
  high_level_command.header = header;
  high_level_command.action = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(high_level_command);  

  estimated_pose_msg = *ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/" + nspace + "/"+self_localization_pose_str, node_handle, ros::Duration(1));
  estimated_speed_msg = *ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/" + nspace + "/"+self_localization_speed_str, node_handle, ros::Duration(1));

  
  reference_pose.pose.position = estimated_pose_msg.pose.position;

  // Extract target yaw
  std::string arguments=getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["angle"].IsDefined())
  {
    angle=config_file["angle"].as<double>() * M_PI/180;
    q_rot.setRPY(0, 0, angle);
    reference_pose.pose.orientation.w = q_rot.getW();
    reference_pose.pose.orientation.x = q_rot.getX();
    reference_pose.pose.orientation.y = q_rot.getY();
    reference_pose.pose.orientation.z = q_rot.getZ();
  }
  else
  {
    if(config_file["relative_angle"].IsDefined())
    {
      angle=config_file["relative_angle"].as<double>() * M_PI/180;
      if(!(estimated_pose_msg.pose.orientation.w == 0 && estimated_pose_msg.pose.orientation.x == 0 && estimated_pose_msg.pose.orientation.y == 0 && estimated_pose_msg.pose.orientation.z == 0)){
          angle = angle + atan2(2.0 * (estimated_pose_msg.pose.orientation.z * estimated_pose_msg.pose.orientation.w + estimated_pose_msg.pose.orientation.x * estimated_pose_msg.pose.orientation.y) , 
                              - 1.0 + 2.0 * (estimated_pose_msg.pose.orientation.w * estimated_pose_msg.pose.orientation.w + estimated_pose_msg.pose.orientation.x * estimated_pose_msg.pose.orientation.x));    
      }
      if(angle > 2* M_PI || angle < -2*M_PI)
      {
        angle=angle*180/M_PI;
        angle= fmod(angle,360);
        angle=angle*M_PI/180;
      }
      q_rot.setRPY(0, 0, angle);
      reference_pose.pose.orientation.w = q_rot.getW();
      reference_pose.pose.orientation.x = q_rot.getX();
      reference_pose.pose.orientation.y = q_rot.getY();
      reference_pose.pose.orientation.z = q_rot.getZ();
    }
  }

}

void BehaviorRotateWithPidControl::onDeactivate(){
  std_msgs::Header header;
  header.frame_id = "behavior_rotate_with_pid_control";

  aerostack_msgs::FlightActionCommand msg;
  msg.header = header;
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(msg);
  
  setControlModeClientSrv.shutdown();
  self_localization_pose_sub.shutdown();
  command_high_level_pub.shutdown();
  motion_reference_speed_pub.shutdown();
  motion_reference_pose_pub.shutdown();
}

void BehaviorRotateWithPidControl::onExecute(){
  setControlMode(aerostack_msgs::MotionControlMode::POSE);
  motion_reference_pose_pub.publish(reference_pose);
  std_msgs::Header header;
  header.frame_id = "behavior_rotate_with_pid_control";
  high_level_command.header = header;
  high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
  command_high_level_pub.publish(high_level_command); 
  if (quadrotor_moving){
    if (checkQuadrotorStopped()){
      quadrotor_moving = false;
      setControlMode(aerostack_msgs::MotionControlMode::POSE);
      motion_reference_pose_pub.publish(reference_pose);
      //high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
      //command_high_level_pub.publish(high_level_command);
    }
  }  
}

bool BehaviorRotateWithPidControl::checkQuadrotorStopped(){
  if (abs(estimated_speed_msg.twist.linear.x) <= 0.15 && abs(estimated_speed_msg.twist.linear.y) <= 0.15 && abs(estimated_speed_msg.twist.linear.z) <= 0.15){
    return true;
  }else{
    return false;
  }
}

void BehaviorRotateWithPidControl::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){
  estimated_speed_msg = msg;
}
void BehaviorRotateWithPidControl::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}
void BehaviorRotateWithPidControl::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

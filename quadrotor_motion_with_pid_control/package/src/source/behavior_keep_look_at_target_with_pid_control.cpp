/*!********************************************************************************
 * \brief     behavior_keep_look_at_target_with_pid_control implementation
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

#include "../include/behavior_keep_look_at_target_with_pid_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorKeepLookAtTargetWithPidControl behavior;
  behavior.start();
  return 0;
}


BehaviorKeepLookAtTargetWithPidControl::BehaviorKeepLookAtTargetWithPidControl() : BehaviorExecutionManager() 
{ 
  setName("keep_look_at_target_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}


BehaviorKeepLookAtTargetWithPidControl::~BehaviorKeepLookAtTargetWithPidControl() {}


void BehaviorKeepLookAtTargetWithPidControl::onConfigure()
{ 
  node_handle = getNodeHandle();
  nspace = getNamespace();

  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic"                   ,self_localization_pose_str                ,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~controllers_topic"                      ,command_high_level_str                    ,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_pose_topic"  	        ,motion_reference_pose_str                 ,"motion_reference/pose");
  ros_utils_lib::getPrivateParam<std::string>("~set_control_mode_service_name"	        ,set_control_mode_srv                      ,"set_control_mode");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"                           ,status_str                                ,"self_localization/flight_state");
  ros_utils_lib::getPrivateParam<std::string>("~shared_robot_position_channel_topic"    ,shared_robot_position_channel_topic_str   ,"shared_robot_positions_channel");

  status_sub = node_handle.subscribe("/" + nspace + "/" + status_str, 1, &BehaviorKeepLookAtTargetWithPidControl::statusCallBack, this);
}


bool BehaviorKeepLookAtTargetWithPidControl::checkSituation()
{
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  //Quadrotor is FLYING
  if((status_msg.state == aerostack_msgs::FlightState::FLYING) || (status_msg.state == aerostack_msgs::FlightState::HOVERING))
  {
    rsp.situation_occurs = true;
  }
  else
  {
    rsp.situation_occurs = false;
  }
  return rsp.situation_occurs;
}


void BehaviorKeepLookAtTargetWithPidControl::onActivate()
{
  // Subscribers
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/" + self_localization_pose_str, 1, &BehaviorKeepLookAtTargetWithPidControl::selfLocalizationPoseCallBack, this);
  shared_robot_pos_sub = node_handle.subscribe("/" + shared_robot_position_channel_topic_str, 1, &BehaviorKeepLookAtTargetWithPidControl::sharedRobotPositionCallback, this);
  // Publishers
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + command_high_level_str, 1, true);
  motion_reference_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/" + motion_reference_pose_str, 1, true);
  // Service
  set_control_mode_client_srv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/" + set_control_mode_srv);
  // Parameters
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["target_name"].IsDefined()) TARGET_NAME = config_file["target_name"].as<std::string>(); // string
  else
  {
    std::cout << "Error: target_name is a mandatory parameter" <<  std::endl;
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::PROCESS_FAILURE);
  }
}


void BehaviorKeepLookAtTargetWithPidControl::onExecute()
{ 
  if(shared_robot_pos_received && estimated_pose_msg_received)
  {
    // Calculate the orientation of the tracker drone (yaw)
    orientation_quaternion.setRPY(0, 0, atan2((shared_drone_coord.position.y - estimated_pose_msg.pose.position.y), (shared_drone_coord.position.x - estimated_pose_msg.pose.position.x)));
    path_point.pose.orientation.w = orientation_quaternion.getW();
    path_point.pose.orientation.x = orientation_quaternion.getX();
    path_point.pose.orientation.y = orientation_quaternion.getY();
    path_point.pose.orientation.z = orientation_quaternion.getZ();
    // Change controller mode
    if(!setControlMode(aerostack_msgs::MotionControlMode::SPEED_3D))
    {
      std::cout << "Error: Is not possible change controller mode" << std::endl;
      BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::PROCESS_FAILURE);
    }  
    // Send trajectory and orientation
    path_point.header.frame_id = "behavior_keep_look_at_target_with_pid_control";
    // Publish orientation (yaw) 
    motion_reference_pose_pub.publish(path_point);

    high_level_command.header.frame_id = "behavior_keep_look_at_target_with_pid_control";
    high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
    command_high_level_pub.publish(high_level_command);
  }
}


void BehaviorKeepLookAtTargetWithPidControl::onDeactivate()
{
  if(shared_robot_pos_received && estimated_pose_msg_received) 
  { 
    std::cout << "Leaving behavior keep look at target with pid control..." << std::endl;
    aerostack_msgs::FlightActionCommand msg;
    msg.header.frame_id = "behavior_keep_look_at_target_with_pid_control";
    msg.action = aerostack_msgs::FlightActionCommand::HOVER;
    command_high_level_pub.publish(msg);
    // Subscribers
    self_localization_pose_sub.shutdown();
    // Publishers
    command_high_level_pub.shutdown();
    motion_reference_pose_pub.shutdown();
    // Services
    set_control_mode_client_srv.shutdown();
    std::cout << "Behavior keep look at target with pid control is deactivated" << std::endl;
  }
}


void BehaviorKeepLookAtTargetWithPidControl::checkGoal()
{
  
}


void BehaviorKeepLookAtTargetWithPidControl::checkProgress() 
{ 
 
}


void BehaviorKeepLookAtTargetWithPidControl::checkProcesses() 
{ 
  
}


bool BehaviorKeepLookAtTargetWithPidControl::setControlMode(int new_control_mode)
{
  // Prepare service message
  aerostack_msgs::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.controlMode.mode = new_control_mode;
  // Use service
  if (set_control_mode_client_srv.call(set_control_mode_srv))
  {
    return set_control_mode_srv.response.ack;
  }
  else
  {
    return false;
  }
}


void BehaviorKeepLookAtTargetWithPidControl::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg)
{
  estimated_pose_msg = msg;
  estimated_pose_msg_received = true;
}


void BehaviorKeepLookAtTargetWithPidControl::statusCallBack(const aerostack_msgs::FlightState &msg)
{
  status_msg = msg;
}


void BehaviorKeepLookAtTargetWithPidControl::sharedRobotPositionCallback(const aerostack_msgs::SharedRobotPosition &msg)
{ 
  shared_robot_pos_received = true;
  if((time != msg.time) && (TARGET_NAME.compare(nspace) != 0) && (msg.sender.compare(TARGET_NAME) == 0))
  {
    shared_drone_coord = msg;
    time = msg.time;
  }
}



    
    
            
   
   

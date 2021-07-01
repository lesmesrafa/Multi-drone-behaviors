/*!********************************************************************************
 * \brief     behavior_keep_target_altitude_with_pid_control implementation
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

#include "../include/behavior_keep_target_altitude_with_pid_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorKeepTargetAltitudeWithPidControl behavior;
  behavior.start();
  return 0;
}


BehaviorKeepTargetAltitudeWithPidControl::BehaviorKeepTargetAltitudeWithPidControl() : BehaviorExecutionManager() 
{ 
  setName("keep_target_altitude_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}


BehaviorKeepTargetAltitudeWithPidControl::~BehaviorKeepTargetAltitudeWithPidControl() {}


void BehaviorKeepTargetAltitudeWithPidControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();
  
  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic"                   ,self_localization_pose_str                ,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~controllers_topic"                      ,command_high_level_str                    ,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_speed_topic" 	        ,motion_reference_speed_str                ,"motion_reference/speed");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_pose_topic"  	        ,motion_reference_pose_str                 ,"motion_reference/pose");
  ros_utils_lib::getPrivateParam<std::string>("~set_control_mode_service_name"	        ,set_control_mode_srv                      ,"set_control_mode");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"                           ,status_str                                ,"self_localization/flight_state");
  ros_utils_lib::getPrivateParam<std::string>("~shared_robot_position_channel_topic"    ,shared_robot_position_channel_topic_str   ,"shared_robot_positions_channel");

  status_sub = node_handle.subscribe("/" + nspace + "/" + status_str, 1, &BehaviorKeepTargetAltitudeWithPidControl::statusCallBack, this);
}


void BehaviorKeepTargetAltitudeWithPidControl::onActivate()
{  
  // Subscribers
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/" + self_localization_pose_str, 1, &BehaviorKeepTargetAltitudeWithPidControl::selfLocalizationPoseCallBack, this);
  shared_robot_pos_sub = node_handle.subscribe("/" + shared_robot_position_channel_topic_str, 1, &BehaviorKeepTargetAltitudeWithPidControl::sharedRobotPositionCallback, this);
  // Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/" + motion_reference_speed_str, 1, true);
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
  if(config_file["shift"].IsDefined()) SHIFT = config_file["shift"].as<double>(); // meters
  else SHIFT = 0.0; // meters
  if(config_file["maximum_speed"].IsDefined()) 
  {
    MAXIMUM_SPEED = std::abs(config_file["maximum_speed"].as<double>()); // m/s
    enable_speed3D = true;
  }
}


void BehaviorKeepTargetAltitudeWithPidControl::onDeactivate()
{ 
  if(shared_robot_pos_received && estimated_pose_msg_received){
    std::cout << "Leaving behavior keep target altitude with pid control..." << std::endl;
    high_level_command.header.frame_id = "keep_target_altitude_with_pid_control";
    high_level_command.action = aerostack_msgs::FlightActionCommand::HOVER;
    motion_reference_speed.twist.linear.z = 0.0;
    motion_reference_speed_pub.publish(motion_reference_speed);
    command_high_level_pub.publish(high_level_command);
    // Subscribers
    self_localization_pose_sub.shutdown();
    // Publishers
    motion_reference_speed_pub.shutdown();
    command_high_level_pub.shutdown();
    motion_reference_pose_pub.shutdown();
    // Services
    set_control_mode_client_srv.shutdown();
    std::cout << "Behavior keep target altitude with pid control is deactivated" << std::endl;
  }
}

 
void BehaviorKeepTargetAltitudeWithPidControl::onExecute()
{ 
  if(shared_robot_pos_received && estimated_pose_msg_received)
  {
    if(shared_drone_coord.position.z > 0.5 || shared_drone_coord.position.z + SHIFT > 0.5) altitude_point_to_achieve = shared_drone_coord.position.z + SHIFT;
    else altitude_point_to_achieve = 0.5;   
    // Change controller mode
    if(enable_speed3D && std::abs(estimated_pose_msg.pose.position.z - altitude_point_to_achieve) > 0.1)
    {
      if(!setControlMode(aerostack_msgs::MotionControlMode::SPEED_3D))
      {
        std::cout << "Error: Is not possible change controller mode" << std::endl;
        BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::PROCESS_FAILURE);
      }  
      
      distxyz = sqrt(pow(shared_drone_coord.position.x - estimated_pose_msg.pose.position.x, 2) + pow(shared_drone_coord.position.y - estimated_pose_msg.pose.position.y, 2) + pow(altitude_point_to_achieve - estimated_pose_msg.pose.position.z, 2));
      distz = altitude_point_to_achieve - estimated_pose_msg.pose.position.z;
      if(altitude_point_to_achieve < estimated_pose_msg.pose.position.z) 
      { 
        motion_reference_speed.twist.linear.z = -(std::abs((MAXIMUM_SPEED * distz) / distxyz));
      }
      else
      { 
        motion_reference_speed.twist.linear.z = (MAXIMUM_SPEED * distz) / distxyz;
      }
      motion_reference_speed_pub.publish(motion_reference_speed);
    }
    else
    { 
      if(!setControlMode(aerostack_msgs::MotionControlMode::GROUND_SPEED))
      {
        std::cout << "Error: Is not possible change controller mode" << std::endl;
        BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::PROCESS_FAILURE);
      }
      path_point.pose.position.z = altitude_point_to_achieve;
      motion_reference_pose_pub.publish(path_point);
    }
    high_level_command.header.frame_id = "keep_target_altitude_with_pid_control";
    high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
    command_high_level_pub.publish(high_level_command); 
  }
}


bool BehaviorKeepTargetAltitudeWithPidControl::checkSituation()
{
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  //Quadrotor is FLYING
  if((status_msg.state == aerostack_msgs::FlightState::FLYING) || (status_msg.state == aerostack_msgs::FlightState::HOVERING))
  {
    rsp.situation_occurs = true;
  }
  else{
    rsp.situation_occurs = false;
  }
  return rsp.situation_occurs;
}


void BehaviorKeepTargetAltitudeWithPidControl::checkGoal()
{   
    
}


void BehaviorKeepTargetAltitudeWithPidControl::checkProgress() 
{ 
 
}


void BehaviorKeepTargetAltitudeWithPidControl::checkProcesses() 
{ 

}


bool BehaviorKeepTargetAltitudeWithPidControl::setControlMode(int new_control_mode)
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


void BehaviorKeepTargetAltitudeWithPidControl::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg)
{
  estimated_pose_msg_received = true;
  estimated_pose_msg = msg;
}


void BehaviorKeepTargetAltitudeWithPidControl::statusCallBack(const aerostack_msgs::FlightState &msg)
{
  status_msg = msg;
}


void BehaviorKeepTargetAltitudeWithPidControl::sharedRobotPositionCallback(const aerostack_msgs::SharedRobotPosition &msg)
{ 
  
  if((time != msg.time) && (TARGET_NAME.compare(nspace) != 0) && (msg.sender.compare(TARGET_NAME) == 0))
  { 
    shared_drone_coord = msg;
    time = msg.time;
    shared_robot_pos_received = true;
  }
}
 

    
    
            
   
   

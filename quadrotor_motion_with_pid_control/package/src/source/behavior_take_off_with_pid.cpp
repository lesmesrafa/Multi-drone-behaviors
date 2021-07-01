/*!********************************************************************************
 * \brief     Take off behavior implementation 
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

#include "../include/behavior_take_off_with_pid.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorTakeOffWithPid behavior;
  behavior.start();
  return 0;
}

BehaviorTakeOffWithPid::BehaviorTakeOffWithPid() : BehaviorExecutionManager(){ 
  setName("take_off_with_pid");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorTakeOffWithPid::~BehaviorTakeOffWithPid(){}

void BehaviorTakeOffWithPid::onConfigure(){ 
  nh = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~battery_topic", battery_topic);
  ros::param::get("~flight_state_topic", state_str);

  ros_utils_lib::getPrivateParam<std::string>("~estimated_speed_topic"	    	, estimated_speed_topic			,"self_localization/speed");
  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    	, estimated_pose_topic 			,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~flight_action_topic"		  	, flight_action_topic    		,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_speed_topic" 	, motion_reference_speed_topic  ,"motion_reference/speed");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_pose_topic"  	, motion_reference_pose_topic   ,"motion_reference/pose");
  ros_utils_lib::getPrivateParam<std::string>("~set_control_mode_service_name"	, set_control_mode_service_name ,"set_control_mode");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"					, status_str 					,"self_localization/flight_state");

  ros::param::get("~status_topic", status_str);
  set_control_mode_client_srv_ = nh.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/" + set_control_mode_service_name);
  pose_sub_ = nh.subscribe("/" + nspace + "/" + estimated_pose_topic ,1,&BehaviorTakeOffWithPid::poseCallback,this);
  speeds_sub_ = nh.subscribe("/" + nspace + "/" + estimated_speed_topic,1,&BehaviorTakeOffWithPid::speedsCallback,this);
  //flight_action_sub = nh.subscribe("/" + nspace + "/" + flight_action_topic,1,&BehaviorTakeOffWithPid::flightActionCallback,this);

  speed_references_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/" + motion_reference_speed_topic, 1);
  pose_references_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/" + motion_reference_pose_topic, 1);

  battery_subscriber = nh.subscribe("/" + nspace + "/"+battery_topic, 1, &BehaviorTakeOffWithPid::batteryCallback, this);
  status_sub = nh.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorTakeOffWithPid::statusCallBack, this);
  flightstate_pub = nh.advertise<aerostack_msgs::FlightState>("/" + nspace + "/"+status_str, 1, true);
  isLow=false;
}

void BehaviorTakeOffWithPid::onActivate(){
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_topic, 1, true);
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
  flight_action_msg_.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
  flight_action_pub.publish(msg);
  activationPosition = position_;
}

void BehaviorTakeOffWithPid::onDeactivate(){
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(msg);
  flight_action_pub.shutdown();
}

void BehaviorTakeOffWithPid::onExecute(){
  if (status_msg.state == aerostack_msgs::FlightState::LANDED || status_msg.state == aerostack_msgs::FlightState::TAKING_OFF || status_msg.state == aerostack_msgs::FlightState::FLYING){
		#if USE_POSE_REFERENCES
		enableDroneControl(aerostack_msgs::MotionControlMode::GROUND_SPEED);
		#else
		enableDroneControl(aerostack_msgs::MotionControlMode::SPEED);
		#endif
		sendAltitudeSpeedReferences(TAKEOFF_SPEED);
    aerostack_msgs::FlightActionCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
    flight_action_pub.publish(msg);
	}
}

bool BehaviorTakeOffWithPid::checkSituation(){
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  if (status_msg.state != aerostack_msgs::FlightState::LANDED && status_msg.state != aerostack_msgs::FlightState::UNKNOWN){
    setErrorMessage("Error: Already flying");
    rsp.situation_occurs = false;
  }else if(isLow){
    setErrorMessage("Error: Battery low, unable to perform action");
    rsp.situation_occurs = false;
  }
  else{
    rsp.situation_occurs = true;
  }
  return rsp.situation_occurs;
}

void BehaviorTakeOffWithPid::batteryCallback(const sensor_msgs::BatteryState& battery) {
	if(battery.percentage * 100 < BATTERY_LOW_THRESHOLD) {
		isLow=true;
	}else{
    isLow=false;
  }
}

void BehaviorTakeOffWithPid::checkGoal(){
  // Check achievement
	if (checkTakeoff()){
		sendAltitudeSpeedReferences(0.0f);
    aerostack_msgs::FlightState msg;
    msg.header.stamp = ros::Time::now();
    msg.state = aerostack_msgs::FlightState::FLYING;
    flightstate_pub.publish(msg);
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
	}
}

void BehaviorTakeOffWithPid::checkProgress(){}

void BehaviorTakeOffWithPid::checkProcesses(){}

bool BehaviorTakeOffWithPid::enableDroneControl(const int& mode){
	// Prepare service message
	aerostack_msgs::SetControlMode set_control_mode_msg;
	set_control_mode_msg.request.controlMode.mode = mode;
	// use service
	if (set_control_mode_client_srv_.call(set_control_mode_msg)){
		return set_control_mode_msg.response.ack;
	}
	else{
		return false;
	}
}

bool BehaviorTakeOffWithPid::checkTakeoff(){
	if (position_.z > activationPosition.z+TAKEOFF_ALTITUDE)
		return true;
	return false;
}

void BehaviorTakeOffWithPid::sendAltitudeSpeedReferences(const double& dz_speed , const double takeoff_altitude){
	geometry_msgs::PoseStamped pose_refs;
	geometry_msgs::TwistStamped speed_refs;
	speed_refs.header.stamp = ros::Time::now();
	pose_refs.header.stamp =  ros::Time::now();
	pose_refs.pose.position.z = takeoff_altitude;
	speed_refs.twist.linear.z = dz_speed;
	speed_references_pub_.publish(speed_refs);
	pose_references_pub_.publish(pose_refs);
}

void BehaviorTakeOffWithPid::speedsCallback(const geometry_msgs::TwistStamped& _msg){
	dz_measure_ = _msg.twist.linear.z;
}

void BehaviorTakeOffWithPid::poseCallback(const geometry_msgs::PoseStamped& _msg){
	position_=_msg.pose.position;
	tf::Quaternion q;
	tf::quaternionMsgToTF(_msg.pose.orientation,q);
	tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll, pitch, yaw);
	roll_ = roll;
	pitch_ = pitch;
}

void BehaviorTakeOffWithPid::flightActionCallback(const aerostack_msgs::FlightActionCommand& _msg){
  flight_action_msg_ = _msg;
}

void BehaviorTakeOffWithPid::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}
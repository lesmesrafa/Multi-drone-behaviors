/*!*******************************************************************************************
 *  \file       quadrotor_pid_controller_process.cpp
 *  \brief      Flight Motion Pid Controller implementation file.
 *  \details    The ROS node "flight motion PID controller" implements a motion controller for multirotors that operates basically in the following way:
 *              The controller receives as input data a motion reference that establishes the movement goal expressed as follows: 
 *              the desired pose <x, y, z, yaw> 
 *              or the desired ground speed <dx, dy>.
 *              As a result, the controller generates a periodic output at a high frequency for the multirotor actuators. 
 *              This output is expressed with the following values <yaw_rate, altitud_rate, pitch, roll>.
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

#include "quadrotor_pid_controller_process.h"

QuadrotorControllerProcess::QuadrotorControllerProcess()
{  

   altitude_control= AltitudeController();
   yaw_control= YawController();
   position_control= PositionController(),
   speed_control= SpeedController();
   started = false;
}

QuadrotorControllerProcess::~QuadrotorControllerProcess()
{}

double QuadrotorControllerProcess::get_moduleRate()
{ 
 return rate;
}

void QuadrotorControllerProcess::ownSetUp()
{   // Configs
    //
    ros::param::get("~robot_namespace", robot_namespace);
    if ( robot_namespace.length() == 0) robot_namespace = "drone1";
    std::cout << "robot_namespace=" <<robot_namespace<< std::endl;

    ros::param::get("~robot_config_path", robot_config_path);
    if ( robot_config_path.length() == 0) robot_config_path = "$(env AEROSTACK_STACK)/configs/"+robot_namespace;
    std::cout << "robot_config_path=" <<robot_config_path<< std::endl;

    ros::param::get("~config_file", config_file);
    if ( config_file.length() == 0) config_file="quadrotor_pid_controller_config.yaml";
    std::cout<<"config_file="<<config_file<<std::endl;

    ros::param::get("~frequency", rate);

    std::string init_control_mode_str;
    ros::param::get("~init_control_mode", init_control_mode_str);
    if ( init_control_mode_str.length() == 0) init_control_mode_str="pose";
    std::cout<<"init_control_mode="<<init_control_mode_str<<std::endl;

    if ( init_control_mode_str.compare("speed") == 0 ) {
        init_control_mode = aerostack_msgs::MotionControlMode::SPEED;
    } else {
        if ( init_control_mode_str.compare("pose") == 0 ) {
            init_control_mode = aerostack_msgs::MotionControlMode::POSE;
        } else {          // "trajectory"
            init_control_mode = aerostack_msgs::MotionControlMode::SPEED;
            throw std::runtime_error("Controller_MidLevel_SpeedLoop::Controller_MidLevel_SpeedLoop, initial control_mode could not be recognized");
        return;
        }
    }

    // Topics
    ros::param::get("~self_localization_pose_topic_name", self_localization_pose_topic_name);
    std::cout<<"self_localization_pose_topic_name="<<self_localization_pose_topic_name<<std::endl;
    //
    ros::param::get("~self_localization_speed_topic_name", self_localization_speed_topic_name);
    std::cout<<"self_localization_speed_topic_name="<<self_localization_speed_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_pose_topic_name", motion_reference_pose_topic_name);
    std::cout<<"motion_reference_pose_topic_name="<<motion_reference_pose_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_assumed_pose_topic_name", motion_reference_assumed_pose_topic_name);
    std::cout<<"motion_reference_assumed_pose_topic_name="<<motion_reference_assumed_pose_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_ground_speed_topic_name", motion_reference_ground_speed_topic_name);
    std::cout<<"motion_reference_ground_speed_topic_name="<<motion_reference_ground_speed_topic_name<<std::endl;
    //
    ros::param::get("~actuator_command_roll_pitch_topic_name", actuator_command_roll_pitch_topic_name);
    std::cout<<"actuator_command_roll_pitch_topic_name="<<actuator_command_roll_pitch_topic_name<<std::endl;
    //
    ros::param::get("~actuator_command_altitude_yaw_topic_name", actuator_command_altitude_yaw_topic_name);
    std::cout<<"actuator_command_altitude_yaw_topic_name="<<actuator_command_altitude_yaw_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_assumed_control_mode_topic_name", motion_reference_assumed_control_mode_topic_name);
    std::cout<<"motion_reference_assumed_control_mode_topic_name="<<motion_reference_assumed_control_mode_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_assumed_ground_speed", motion_reference_assumed_ground_speed);
    std::cout<<"motion_reference_assumed_ground_speed="<<motion_reference_assumed_ground_speed<<std::endl;

    //Service
    ros::param::get("~set_control_mode_service_name", set_control_mode_service_name);
    std::cout<<"set_control_mode_service_name="<<set_control_mode_service_name<<std::endl;
    
    // Load file
    YAML::Node yamlconf;
    try
    {
        yamlconf = YAML::LoadFile(robot_config_path+"/"+config_file);
    }
    catch (std::exception& e)
    {
       std::cerr<<(std::string("[YamlException! Error reading yaml config file.  Caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n")<<std::endl;
       exit(-1);
    }

    altitude_control.setUp(yamlconf);
    yaw_control.setUp(yamlconf);
    position_control.setUp(yamlconf);
    speed_control.setUp(yamlconf);
}

void QuadrotorControllerProcess::ownStart(){    
    std::cout<<"QUADROTOR PID CONTROLLER STARTED"<<std::endl;
    ros::NodeHandle n;
    //Subscribers
    selfLocalizationPose = n.subscribe(self_localization_pose_topic_name, 1, &QuadrotorControllerProcess::selfLocalizationPoseCallback, this);
    selfLocalizationSpeed = n.subscribe(self_localization_speed_topic_name, 1, &QuadrotorControllerProcess::selfLocalizationSpeedCallback, this);
    motionReferencePose = n.subscribe(motion_reference_pose_topic_name, 1, &QuadrotorControllerProcess::motionReferencePoseCallback, this);
    motionReferenceGroundSpeed =  n.subscribe(motion_reference_ground_speed_topic_name, 1, &QuadrotorControllerProcess::motionReferenceGroundSpeedCallback, this);

    // Publishers
    actuator_pitch_roll_pub = n.advertise<geometry_msgs::PoseStamped>(actuator_command_roll_pitch_topic_name, 1);
    actuator_altitude_yaw_pub = n.advertise<geometry_msgs::TwistStamped>(actuator_command_altitude_yaw_topic_name, 1);
    assumedControlMode = n.advertise<aerostack_msgs::MotionControlMode>(motion_reference_assumed_control_mode_topic_name, 1);
    assumedGroundSpeed = n.advertise<geometry_msgs::TwistStamped>(motion_reference_assumed_ground_speed, 1);
    assumedPose = n.advertise<geometry_msgs::PoseStamped>(motion_reference_assumed_pose_topic_name, 1);

    //Actuator control Publishers
    actuator_command_pose = n.advertise<geometry_msgs::PoseStamped>("actuator_command/pose", 1);
    actuator_command_speed = n.advertise<geometry_msgs::TwistStamped>("actuator_command/speed", 1);

    // Service
    set_control_mode_service = n.advertiseService(set_control_mode_service_name,&QuadrotorControllerProcess::setControlMode_Serv_Call,this);
    control_mode = init_control_mode;
    
    setControlMode(control_mode);
    started = true;
}

void QuadrotorControllerProcess::ownStop(){   
    setNavCommandToZero();
    assumedControlMode.shutdown();
    actuator_pitch_roll_pub.shutdown();
    actuator_altitude_yaw_pub.shutdown();
    selfLocalizationPose.shutdown();
    selfLocalizationSpeed.shutdown();
    motionReferencePose.shutdown();
    motionReferenceGroundSpeed.shutdown();
    assumedGroundSpeed.shutdown();
    assumedPose.shutdown();
    set_control_mode_service.shutdown();
    started = false;
}

void QuadrotorControllerProcess::ownRun()
{
    publishControlMode();
    float pitch, roll, dyaw, dz;
    publishControllerReferences();
    getOutput( &pitch, &roll, &dyaw, &dz);
    setNavCommand( pitch, roll, dyaw, dz);

}

bool QuadrotorControllerProcess::isStarted()
{
    return started;
}

bool QuadrotorControllerProcess::resetValues()
{
    tf2::Matrix3x3 m(tf2::Quaternion (self_localization_pose.pose.orientation.x,self_localization_pose.pose.orientation.y,self_localization_pose.pose.orientation.z,self_localization_pose.pose.orientation.w));
    double r, p, yaw;
    m.getRPY(r, p, yaw);

    control_mode = init_control_mode;

    setControlMode(control_mode);

    altitude_control.stop();
    yaw_control.stop();
    position_control.stop();
    speed_control.stop();
    position_control.setReference(self_localization_pose.pose.position.x, self_localization_pose.pose.position.y);
    speed_control.setReference(0.0,0.0);
    yaw_control.setReference(yaw,yaw);
    altitude_control.setReference(self_localization_pose.pose.position.z);

    motion_reference_pose.pose.position.x = self_localization_pose.pose.position.x;
    motion_reference_pose.pose.position.y = self_localization_pose.pose.position.y;
    motion_reference_pose.pose.position.z = self_localization_pose.pose.position.z;

    motion_reference_ground_speed.twist.linear.x = 0.0;
    motion_reference_ground_speed.twist.linear.y = 0.0;

    motion_reference_pose = self_localization_pose;

    publishControllerReferences();
    setNavCommandToZero();

    altitude_control.start();
    yaw_control.start();
    position_control.start();
    speed_control.start();

    return true;
}

void QuadrotorControllerProcess::selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    self_localization_pose = (*msg);
    self_localization_angular_euler = toEulerianAngle(self_localization_pose);
}

void QuadrotorControllerProcess::selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_speed = (*msg);
}

void QuadrotorControllerProcess::motionReferencePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    motion_reference_pose = (*msg);
}

void QuadrotorControllerProcess::motionReferenceGroundSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    motion_reference_ground_speed = (*msg);
}

void QuadrotorControllerProcess::resetActuatorCommands(){
    command_pose.header.stamp = ros::Time::now();
    command_speed.header.stamp = ros::Time::now();

    command_pose.pose.position.x = 0;
    command_pose.pose.position.y = 0;
    command_pose.pose.position.z = 0;
    command_pose.pose.orientation.x = 0;
    command_pose.pose.orientation.y = 0;
    command_pose.pose.orientation.z = 0;
    command_pose.pose.orientation.w = 1;

    command_speed.twist.linear.x = 0;
    command_speed.twist.linear.y = 0;
    command_speed.twist.linear.z = 0;
    
    command_speed.twist.angular.x = 0;
    command_speed.twist.angular.y = 0;
    command_speed.twist.angular.z = 0;

}

void QuadrotorControllerProcess::publishControllerReferences() {
    assumed_ground_speed = motion_reference_ground_speed;
    assumed_pose = motion_reference_pose;

    switch (control_mode){
    case aerostack_msgs::MotionControlMode::SPEED:    //dx, dy, dz, dyaw
        assumed_ground_speed = motion_reference_ground_speed;
        assumed_pose.pose.position.x = 0; assumed_pose.pose.position.y = 0; assumed_pose.pose.position.z = 0;
        assumed_pose.pose.orientation.x = 0; assumed_pose.pose.orientation.y = 0; 
        assumed_pose.pose.orientation.z = 0; assumed_pose.pose.orientation.w = 0;

    break;
    case aerostack_msgs::MotionControlMode::SPEED_3D:    //dx, dy, dz, yaw
        assumed_ground_speed = motion_reference_ground_speed;
        assumed_ground_speed.twist.angular.x = 0; assumed_ground_speed.twist.angular.y = 0; assumed_ground_speed.twist.angular.z = 0;
        assumed_pose.pose.position.x = 0; assumed_pose.pose.position.y = 0; assumed_pose.pose.position.z = 0;
        assumed_pose.pose.orientation = motion_reference_pose.pose.orientation;
        
        break;
    case aerostack_msgs::MotionControlMode::POSE: //x, y, z, yaw
        assumed_pose = motion_reference_pose;
        assumed_ground_speed.twist.angular.x = 0; assumed_ground_speed.twist.angular.y = 0; assumed_ground_speed.twist.angular.z = 0;
        assumed_ground_speed.twist.linear.x = 0; assumed_ground_speed.twist.linear.y = 0; assumed_ground_speed.twist.linear.z = 0;

    break;
    case aerostack_msgs::MotionControlMode::GROUND_SPEED:    //dx, dy, z, yaw
        assumed_ground_speed = motion_reference_ground_speed; assumed_ground_speed.twist.linear.z = 0;
        assumed_ground_speed.twist.angular.x = 0; assumed_ground_speed.twist.angular.y = 0; assumed_ground_speed.twist.angular.z = 0;
        assumed_pose.pose.orientation = motion_reference_pose.pose.orientation;
        assumed_pose.pose.position.x = 0; assumed_pose.pose.position.y = 0; assumed_pose.pose.position.z = motion_reference_pose.pose.position.z;

    break;
    case aerostack_msgs::MotionControlMode::ATTITUDE:    //pitch, roll, z, yaw
        assumed_pose.pose.orientation = motion_reference_pose.pose.orientation;
        assumed_ground_speed.twist.angular.x = 0; assumed_ground_speed.twist.angular.y = 0; assumed_ground_speed.twist.angular.z = 0;
        assumed_ground_speed.twist.linear.x = 0; assumed_ground_speed.twist.linear.y = 0; assumed_ground_speed.twist.linear.z = 0;
        assumed_pose.pose.position.x = 0; assumed_pose.pose.position.y = 0; assumed_pose.pose.position.z = motion_reference_pose.pose.position.z;
        break;
    case aerostack_msgs::MotionControlMode::UNKNOWN:
    default:
        break;
    }

    assumedGroundSpeed.publish(assumed_ground_speed);
    assumedPose.publish(assumed_pose);
    
}

void QuadrotorControllerProcess::publishControlMode() {
    aerostack_msgs::MotionControlMode controlModeMsg2;
    controlModeMsg2.mode = getControlMode();   
    assumedControlMode.publish(controlModeMsg2);     
}

int QuadrotorControllerProcess::publishDroneNavCommand() {
    geometry_msgs::Vector3 thrust;
    thrust.x = 0;
    thrust.y = 0;

    //Standard topics
    actuator_altitude_yaw_pub.publish(actuator_altitude_yaw);
    actuator_pitch_roll_pub.publish(actuator_pitch_roll);

    // Antonio publish topics
    actuator_command_pose.publish(command_pose);
    actuator_command_speed.publish(command_speed);
    return 1;
}

void QuadrotorControllerProcess::setNavCommand(float pitch, float roll, float dyaw, float dz, double time) {
    actuator_altitude_yaw.header.stamp        = ros::Time::now();
    actuator_pitch_roll.header.stamp        = ros::Time::now();

    actuator_altitude_yaw.twist.linear.z = dz;
    actuator_altitude_yaw.twist.angular.z = dyaw;

    tf2::Quaternion m;
    m.setRPY(roll, pitch, 0);

    actuator_pitch_roll.pose.orientation.x = m.getX();
    actuator_pitch_roll.pose.orientation.y = m.getY();
    actuator_pitch_roll.pose.orientation.z = m.getZ();
    actuator_pitch_roll.pose.orientation.w = m.getW();

    publishDroneNavCommand();
    return;
}

void QuadrotorControllerProcess::setNavCommandToZero(void) {
    setNavCommand(0.0,0.0,0.0,0.0,-1.0);
    return;
}
bool QuadrotorControllerProcess::setControlMode(int mode){
    switch (mode) {
    case aerostack_msgs::MotionControlMode::POSE:
        setPositionControl();
        break;
    case aerostack_msgs::MotionControlMode::SPEED:
        setSpeedControl();
        break;
    case aerostack_msgs::MotionControlMode::GROUND_SPEED:
        setGroundSpeedControl();
        break;
    case aerostack_msgs::MotionControlMode::SPEED_3D:
        setSpeed3DControl();
        break;  
    case aerostack_msgs::MotionControlMode::ATTITUDE:
        setAttitudeControl();
        break;                    
    case aerostack_msgs::MotionControlMode::UNKNOWN:
    default:
        return false;
        break;
    }
    control_mode = mode;
    return true;
}

void QuadrotorControllerProcess::getOutput(float *pitch_val, float *roll_val, float *dyaw_val, float *dz_val){
    float pitch = 0.0f, roll= 0.0f, yaw= 0.0f, droll= 0.0f, dpitch= 0.0f, dyaw= 0.0f, x= 0.0f, y= 0.0f ,z= 0.0f, dx= 0.0f, dy= 0.0f, dz= 0.0f ;
    
    resetActuatorCommands();
    yaw = self_localization_angular_euler.z;
    x = self_localization_pose.pose.position.x;
    y = self_localization_pose.pose.position.y;
    z = self_localization_pose.pose.position.z;

    dx = motion_reference_ground_speed.twist.linear.x;
    dy = motion_reference_ground_speed.twist.linear.y;
    dz = motion_reference_ground_speed.twist.linear.z;

    dyaw = motion_reference_ground_speed.twist.angular.z;


    geometry_msgs::Vector3 yaw_position_aux = toEulerianAngle(motion_reference_pose);
    yaw = yaw_position_aux.z;

    switch (control_mode){
    case aerostack_msgs::MotionControlMode::SPEED:    //dx, dy, dz, dyaw
    {
        
        speed_control.setFeedback(current_speed.twist.linear.x, current_speed.twist.linear.y);
        speed_control.setReference(motion_reference_ground_speed.twist.linear.x, motion_reference_ground_speed.twist.linear.y);
        speed_control.getOutput( &pitch, &roll, self_localization_angular_euler.z);

    }
    break;
    case aerostack_msgs::MotionControlMode::SPEED_3D:    //dx, dy, dz, yaw
    {
        speed_control.setFeedback(current_speed.twist.linear.x, current_speed.twist.linear.y);
        speed_control.setReference(motion_reference_ground_speed.twist.linear.x, motion_reference_ground_speed.twist.linear.y);
        speed_control.getOutput( &pitch, &roll, self_localization_angular_euler.z);

        yaw_control.setFeedback(self_localization_angular_euler.z);
        yaw_control.setReference(yaw_position_aux.z, self_localization_angular_euler.z);       
        yaw_control.getOutput(&dyaw);

    }
    break;
    case aerostack_msgs::MotionControlMode::POSE: //x, y, z, yaw
    {
        x = motion_reference_pose.pose.position.x;
        y = motion_reference_pose.pose.position.y;
        z = motion_reference_pose.pose.position.z;

        position_control.setFeedback(self_localization_pose.pose.position.x, self_localization_pose.pose.position.y);
        position_control.setReference(motion_reference_pose.pose.position.x, motion_reference_pose.pose.position.y);
        position_control.getOutput( &dx, &dy);
       
        speed_control.setFeedback(current_speed.twist.linear.x, current_speed.twist.linear.y);
        speed_control.setReference(dx, dy);
        speed_control.getOutput(&pitch, &roll, self_localization_angular_euler.z);

        yaw_control.setFeedback(self_localization_angular_euler.z);
        yaw_control.setReference(yaw_position_aux.z, self_localization_angular_euler.z);       
        yaw_control.getOutput(&dyaw);

        altitude_control.setFeedback(self_localization_pose.pose.position.z);
        altitude_control.setReference(motion_reference_pose.pose.position.z);
        altitude_control.getOutput(&dz);

        
    }
    break;
    case aerostack_msgs::MotionControlMode::GROUND_SPEED:    //dx, dy, z, yaw
    {
        z = motion_reference_pose.pose.position.z;
        
        speed_control.setFeedback(current_speed.twist.linear.x, current_speed.twist.linear.y);
        speed_control.setReference(motion_reference_ground_speed.twist.linear.x, motion_reference_ground_speed.twist.linear.y);
        speed_control.getOutput( &pitch, &roll, self_localization_angular_euler.z);

        yaw_control.setFeedback(self_localization_angular_euler.z);
        yaw_control.setReference(yaw_position_aux.z, self_localization_angular_euler.z);
        yaw_control.getOutput(&dyaw);

        altitude_control.setFeedback(self_localization_pose.pose.position.z);
        altitude_control.setReference(motion_reference_pose.pose.position.z);


        altitude_control.getOutput(&dz);

    }
    break;
    case aerostack_msgs::MotionControlMode::ATTITUDE:    //pitch, roll, z, yaw
    {
        geometry_msgs::Vector3 poseEulerian = toEulerianAngle(motion_reference_pose);
        pitch = poseEulerian.y;
        roll = poseEulerian.x;
        yaw_control.setFeedback(self_localization_angular_euler.z);
        yaw_control.setReference(poseEulerian.z, poseEulerian.z);       
        yaw_control.getOutput(&dyaw);

        altitude_control.setFeedback(self_localization_pose.pose.position.z);
        altitude_control.setReference(motion_reference_pose.pose.position.z);
        altitude_control.getOutput(&dz);
        

        yaw = poseEulerian.z;
        z = motion_reference_pose.pose.position.z;
        

    }
    break;                    
    case aerostack_msgs::MotionControlMode::UNKNOWN:
        setNavCommandToZero();

        pitch = 0.0;
        roll = 0.0;
        dyaw = 0.0;
        dz = 0.0;

    default:
        break;
    }
    // postprocesado

    
    *pitch_val = pitch;
    *roll_val = roll;
    *dyaw_val = dyaw;
    *dz_val = dz;

    command_pose.pose.position.x = x;
    command_pose.pose.position.y = y;
    command_pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(*roll_val,*pitch_val,yaw);

    //std::cout<< "roll:[" << *roll_val << "]   pitch:[" << *pitch_val << "]" <<std::endl;
    
    command_pose.pose.orientation.x = q.getX();
    command_pose.pose.orientation.y = q.getY();
    command_pose.pose.orientation.z = q.getZ();
    command_pose.pose.orientation.w = q.getW();

    command_speed.twist.linear.x = dx;
    command_speed.twist.linear.y = dy;
    command_speed.twist.linear.z = dz;

    command_speed.twist.angular.x = dpitch;
    command_speed.twist.angular.y = droll;
    command_speed.twist.angular.z = dyaw;
}

void QuadrotorControllerProcess::setPositionControl(){
    if(aerostack_msgs::MotionControlMode::POSE != control_mode){
        speed_control.stop();
        position_control.stop();
        altitude_control.stop();
        yaw_control.stop();

        position_control.start();
        position_control.setReference(self_localization_pose.pose.position.x, self_localization_pose.pose.position.y);
        speed_control.start();
        speed_control.setReference(current_speed.twist.linear.x, current_speed.twist.linear.y);
        altitude_control.start();
        altitude_control.setReference(self_localization_pose.pose.position.z);
        yaw_control.start();
        yaw_control.setReference(self_localization_angular_euler.z, self_localization_angular_euler.z);
    }
}

void QuadrotorControllerProcess::setAttitudeControl(){
    if(aerostack_msgs::MotionControlMode::ATTITUDE != control_mode){
        speed_control.stop();
        position_control.stop();
        altitude_control.stop();
        yaw_control.stop();

        position_control.start();
        position_control.setReference(self_localization_pose.pose.position.x, self_localization_pose.pose.position.y);            
        altitude_control.start();
        altitude_control.setReference(self_localization_pose.pose.position.z);  
        yaw_control.start();
        yaw_control.setReference(self_localization_angular_euler.z, self_localization_angular_euler.z);              
    }
}

void QuadrotorControllerProcess::setSpeedControl(){
    if(aerostack_msgs::MotionControlMode::SPEED != control_mode){
        speed_control.stop();
        position_control.stop();
        altitude_control.stop();
        yaw_control.stop();

        speed_control.start();
        speed_control.setReference(current_speed.twist.linear.x, current_speed.twist.linear.y);        
    }
}

void QuadrotorControllerProcess::setGroundSpeedControl(){
    if(aerostack_msgs::MotionControlMode::GROUND_SPEED != control_mode){
        speed_control.stop();
        position_control.stop();
        altitude_control.stop();
        yaw_control.stop();    

        speed_control.start();
        speed_control.setReference(current_speed.twist.linear.x, current_speed.twist.linear.y);  
        altitude_control.start();
        altitude_control.setReference(self_localization_pose.pose.position.z); 
        yaw_control.start();
        yaw_control.setReference(self_localization_angular_euler.z, self_localization_angular_euler.z);                              
    }
}
void QuadrotorControllerProcess::setSpeed3DControl(){
    if(aerostack_msgs::MotionControlMode::SPEED_3D != control_mode){
        speed_control.stop();
        position_control.stop();
        altitude_control.stop();
        yaw_control.stop();

        speed_control.start();
        speed_control.setReference(current_speed.twist.linear.x, current_speed.twist.linear.y);
        yaw_control.start();
        yaw_control.setReference(self_localization_angular_euler.z, self_localization_angular_euler.z);                
    }
}

bool QuadrotorControllerProcess::setControlMode_Serv_Call(aerostack_msgs::SetControlMode::Request& request, aerostack_msgs::SetControlMode::Response& response) {
    int new_control_mode;
    switch (request.controlMode.mode) {
    case aerostack_msgs::MotionControlMode::POSE:
        new_control_mode = aerostack_msgs::MotionControlMode::POSE;
        break;
    case aerostack_msgs::MotionControlMode::SPEED:
        new_control_mode = aerostack_msgs::MotionControlMode::SPEED;
        break;
    case aerostack_msgs::MotionControlMode::GROUND_SPEED:
        new_control_mode = aerostack_msgs::MotionControlMode::GROUND_SPEED;
        break;
    case aerostack_msgs::MotionControlMode::SPEED_3D:
        new_control_mode = aerostack_msgs::MotionControlMode::SPEED_3D;
        break;    
    case aerostack_msgs::MotionControlMode::ATTITUDE:
        new_control_mode = aerostack_msgs::MotionControlMode::ATTITUDE;
        break;                
    case aerostack_msgs::MotionControlMode::UNKNOWN:
    default:
        new_control_mode = aerostack_msgs::MotionControlMode::UNKNOWN;
        break;
    }
    response.ack = setControlMode(new_control_mode);
    return response.ack;
}

geometry_msgs::Vector3 QuadrotorControllerProcess::toEulerianAngle(geometry_msgs::PoseStamped q){
    geometry_msgs::Vector3 result;
    if(q.pose.orientation.w == 0 && q.pose.orientation.x == 0 && q.pose.orientation.y == 0 && q.pose.orientation.z == 0){
        result.x   = 0; //Roll
        result.y = 0;   //Pitch
        result.z = 0;   //Yaw
    }else{
        result.x  = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.y + q.pose.orientation.w * q.pose.orientation.x) , 1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.y));
        result.y = asin(2.0 * (q.pose.orientation.y * q.pose.orientation.w - q.pose.orientation.z * q.pose.orientation.x));
        result.z   = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.y) , - 1.0 + 2.0 * (q.pose.orientation.w * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.x));    
    }
    return result;
}


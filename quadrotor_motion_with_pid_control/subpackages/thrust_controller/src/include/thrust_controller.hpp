#pragma once

#include "ros/ros.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/Thrust.h"
#include <vector>
#include <iostream>
#include "tf/transform_datatypes.h"
#include "tf/tf.h"
#include <string>
#include <yaml-cpp/yaml.h>
#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"
#include "aerostack_msgs/FlightActionCommand.h"
#include "robot_process.h"
#include "std_msgs/Float32MultiArray.h"

#define GRAVITY_CONSTANT 9.81f

#define MAX_THRUST_ (GRAVITY_CONSTANT*mass_*2.0f)
#define MIN_THRUST_ (GRAVITY_CONSTANT*mass_*0.5f)
#define DEBUG 0


class ThrustController :public RobotProcess {

private:
    double mass_ = 1.0f;
	const float antiwindup_limit_ = 500;
    float Kp_ = 9.0f, Kd_ = 0.0 , Ki = 0.01f;

public:
    ThrustController(){};
private:
    void ownSetUp();
    void ownStart(){};
    void ownStop(){};
    void ownRun();

private:
    ros::Subscriber altitude_rate_yaw_rate_sub_; // dz reference¡    
    void altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& );
    float dz_reference_;

    ros::Subscriber pose_sub_; // pose     
    void poseCallback(const geometry_msgs::PoseStamped& );
    geometry_msgs::Point position_;
    double roll_ = 0.0f, pitch_ = 0.0f;
    
    ros::Subscriber speeds_sub_; // speed measures    
    void speedsCallback(const geometry_msgs::TwistStamped& );
    float dz_measure_;
    
    ros::Subscriber flight_action_sub;
    void flightActionCallback(const aerostack_msgs::FlightActionCommand& _msg){
        flight_action_msg_ = _msg;
    };
    aerostack_msgs::FlightActionCommand flight_action_msg_;


    ros::Publisher thrust_pub_; // thrust_pub
    mavros_msgs::Thrust thrust_msg_;

    void computeThrust(double );
    void publishThrust();


    #if DEBUG == 1
    private:
    ros::Publisher thrust_debugger_pub_;
 	std_msgs::Float32MultiArray thrust_debugger_values_msg_;

    ros::Subscriber roll_pitch_yaw_rate_thrust_sub_;
    ros::Publisher roll_pitch_yaw_rate_thrust_pub_; // TODO DELETE THIS

    void rollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrust& );    
    
    #endif



};


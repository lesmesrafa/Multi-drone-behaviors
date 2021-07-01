#include "thrust_controller.hpp"

void ThrustController::ownSetUp()
{
 	static ros::NodeHandle nh;
	
	std::string n_space ;
	std::string estimated_speed_topic;
	std::string estimated_pose_topic;
	std::string altitude_rate_yaw_rate_topic;
	std::string thrust_topic;
	std::string flight_action_topic;
	std::string robot_config_path;
	std::string yaml_config_file;

	ros_utils_lib::getPrivateParam<double>	   ("~uav_mass"						, mass_						    ,1);
	ros_utils_lib::getPrivateParam<std::string>("~namespace"					, n_space						,"drone1");
	ros_utils_lib::getPrivateParam<std::string>("~estimated_speed_topic"	    , estimated_speed_topic			,"self_localization/speed");
	ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    , estimated_pose_topic 			,"self_localization/pose");
	ros_utils_lib::getPrivateParam<std::string>("~altitude_rate_yaw_rate_topic"	, altitude_rate_yaw_rate_topic	,"actuator_command/altitude_rate_yaw_rate");
	ros_utils_lib::getPrivateParam<std::string>("~thrust_topic"					, thrust_topic					,"actuator_command/thrust");
	ros_utils_lib::getPrivateParam<std::string>("~flight_action_topic"		    , flight_action_topic    		,"actuator_command/flight_action");
	ros_utils_lib::getPrivateParam<std::string>("~robot_config_path"		    , robot_config_path    			,"configs/"+n_space);
	ros_utils_lib::getPrivateParam<std::string>("~yaml_config_file"		   		, yaml_config_file    			,"quadrotor_pid_controller_config.yaml");

	std::cout << "uav_mass = " << mass_ << std::endl;
	
	altitude_rate_yaw_rate_sub_ = nh.subscribe("/" + n_space + "/" + altitude_rate_yaw_rate_topic,1,&ThrustController::altitudeRateYawRateCallback,this);
  	pose_sub_   = nh.subscribe("/" + n_space + "/" + estimated_pose_topic ,1,&ThrustController::poseCallback,this);
	speeds_sub_ = nh.subscribe("/" + n_space + "/" + estimated_speed_topic,1,&ThrustController::speedsCallback,this);

	flight_action_sub = nh.subscribe("/" + n_space + "/" + flight_action_topic,1,&ThrustController::flightActionCallback,this);
    
	thrust_pub_ = nh.advertise<mavros_msgs::Thrust>("/" + n_space + "/" + thrust_topic,1);

    thrust_msg_.thrust = 0;
	
	// Load file
    YAML::Node yamlconf;
    try
    {
        yamlconf = YAML::LoadFile(robot_config_path+"/"+yaml_config_file);
    }
    catch (std::exception& e)
    {
       std::cout<<"Yaml config file does not exist in path: "<<robot_config_path<<"/"<<yaml_config_file<<" . Taking default values"<<std::endl;
    }
	if(yamlconf["thrust_controller"]){
		Kp_ = yamlconf["thrust_controller"]["kp"].as<float>();
		Ki  = yamlconf["thrust_controller"]["ki"].as<float>();
		Kd_ = yamlconf["thrust_controller"]["kd"].as<float>();
	}

	//Code for publishing thrust value in a way that can be compared using rqt_plot tool (Only for debugging purposes
	#if DEBUG == 1
    thrust_debugger_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/"+n_space+"/"+"debug/thrust_controller",1);
	thrust_debugger_values_msg_.data = std::vector<float>(2);
	std_msgs::MultiArrayDimension dim;
	dim.label = "ThrustSignal";
	dim.size = 2;
	dim.stride = 1;
	thrust_debugger_values_msg_.layout.dim.emplace_back(dim);
    roll_pitch_yaw_rate_thrust_sub_ = nh.subscribe("/"+n_space+"/"+"actuator_command/roll_pitch_yaw_rate_thrust_test",1,
												    &ThrustController::rollPitchYawRateThrustCallback,this);
	roll_pitch_yaw_rate_thrust_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/"+n_space+"/"+"actuator_command/roll_pitch_yaw_rate_thrust",1);
	#endif
}

void ThrustController::computeThrust(double dz_reference){

	// Initialize static variables
	static ros::Time prev_time = ros::Time::now();
	static float accum_error = 0.0f;
	static float last_dz_error = 0.0f;
	static float last_reference = 0.0f;
	static ros_utils_lib::MovingAverageFilter dz_derivative_filtering(0.85);

	float& thrust = thrust_msg_.thrust;
	double dtime = (ros::Time::now()-prev_time).toSec();
	prev_time = ros::Time::now();
	
	double feed_forward = 0.0f;
	// double feed_forward = dz_reference_ - last_reference;

	float dz_error = (dz_reference- dz_measure_);
	float dz_derivative_error = (dz_error-last_dz_error)/(dtime+1e-9);
	// dz_derivative_error = dz_derivative_filtering.filterValue(dz_derivative_error);

	last_reference = dz_reference;
	last_dz_error = dz_error;

	accum_error += dz_error;
	accum_error = (accum_error >   antiwindup_limit_)?   antiwindup_limit_ : accum_error;
	accum_error = (accum_error < - antiwindup_limit_)? - antiwindup_limit_ : accum_error;
	
	thrust = mass_ * (GRAVITY_CONSTANT + feed_forward +Kp_ *dz_error + Ki*accum_error + Kd_*dz_derivative_error);
	thrust = thrust /(cos(pitch_)*cos(roll_)); // project Thrust in z axis

	thrust = (thrust < MIN_THRUST_)? MIN_THRUST_ : thrust; // LOW LIMIT THRUST IN [0, MAX_THRUST]
	thrust = (thrust > MAX_THRUST_)? MAX_THRUST_ : thrust; // HIGH LIMIT THRUST IN [0, MAX_THRUST]
	
	//std::cout << "dz_reference = " << dz_reference << std::endl;
	//std::cout << "dz_error = " << dz_error << std::endl;
	//std::cout << "thrust = " << thrust << std::endl;

}

void ThrustController::ownRun(){	
	computeThrust(dz_reference_);	
	publishThrust();
}

void ThrustController::publishThrust(){
	thrust_pub_.publish(thrust_msg_);
}

// __________________________________CALLBACKS_________________________________________

void ThrustController::altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& _msg){
	thrust_msg_.header = _msg.header;
	dz_reference_ = _msg.twist.linear.z;
}

void ThrustController::speedsCallback(const geometry_msgs::TwistStamped& _msg){
	dz_measure_ = _msg.twist.linear.z;
}

void ThrustController::poseCallback(const geometry_msgs::PoseStamped& _msg){
	position_=_msg.pose.position;
	tf::Quaternion q;
	tf::quaternionMsgToTF(_msg.pose.orientation,q);
	tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll, pitch, yaw);
	roll_ = roll;
	pitch_ = pitch;
 }	


/*------------------------	DEBUGGING FUNCTIONS --------------------------*/
#if DEBUG == 1

	void ThrustController::rollPitchYawRateThrustCallback(const mav_msgs::RollPitchYawrateThrust& _msg){

		std::cout << "dz_measure :"<< dz_measure_<< std::endl ;
		std::cout << "dz_reference :"<< dz_reference_<< std::endl ;

		thrust_debugger_values_msg_.data[0] = _msg.thrust.z;
		thrust_debugger_values_msg_.data[1] = thrust_msg_.thrust;
		thrust_debugger_pub_.publish(thrust_debugger_values_msg_);	

		static mav_msgs::RollPitchYawrateThrust msg;
		msg = _msg;
		msg.thrust.z = thrust_msg_.thrust;

		std::cout << "prev_thrust :"<< _msg.thrust.z<< std::endl ;
		std::cout << "new_thrust :"<< msg.thrust.z<< std::endl ;

		roll_pitch_yaw_rate_thrust_pub_.publish(msg);
	}

#endif


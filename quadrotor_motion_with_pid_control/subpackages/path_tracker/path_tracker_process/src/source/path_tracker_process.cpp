/*!*******************************************************************************************
 *  \file       path_tracker_process.cpp
 *  \brief      Path Tracker implementation file.
 *  \details    The ROS node “path_tracker” implements an algorithm for path tracking for autonomous robots that operates in the following way:
 *              The node receives as input data a path to track as a sequence of point values. 
 *              As a result, the controller generates a periodic output for a motion controller with reference speed and reference.
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

#include "path_tracker_process.h"

//Constructor
PathTrackerProcess::PathTrackerProcess(){
    std::cout << "Constructor: PathTrackerProcess" << std::endl;
}

//Destructor
PathTrackerProcess::~PathTrackerProcess() {}

double PathTrackerProcess::get_moduleRate()
{ 
 return rate;
}

bool PathTrackerProcess::readConfigs(std::string configFile){
    try{
    // Load file
    YAML::Node yamlconf = YAML::LoadFile(configFile);

    /*********************************  Constant Needed ************************************************/

    v_maxxy = fabs(yamlconf["vxy_max"].as<float>());
    v_maxz = fabs(yamlconf["vz_max"].as<float>());
    precision = fabs(yamlconf["precision"].as<float>());
    path_facing = fabs(yamlconf["path_facing"].as<bool>());
    }
    catch (std::exception& e)
    {
       std::cerr<<(std::string("[YamlException! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n")<<std::endl;
       exit(-1);
    }
    return true;
}

void PathTrackerProcess::ownRun(){
    if(number_of_points > 0) getOutput();
}

void PathTrackerProcess::ownSetUp(){

    ros::param::get("~robot_namespace", robot_namespace);
    if ( robot_namespace.length() == 0){
        robot_namespace = "drone1";
    }
    std::cout << "robot_namespace=" <<robot_namespace<< std::endl; 
      
    ros::param::get("~robot_config_path", robot_config_path);
    if ( robot_config_path.length() == 0){
        robot_config_path = "$(env AEROSTACK_STACK)/configs/"+robot_namespace;
    }
    std::cout << "robot_config_path=" <<robot_config_path<< std::endl;

    ros::param::get("~config_file", config_file);
    if ( config_file.length() == 0)
    {
        config_file="path_tracker.yaml";
    }

    std::cout<<"config_file="<<config_file<<std::endl;
    ros::param::get("~frequency", rate);

    bool readConfigsBool = readConfigs(robot_config_path+"/"+config_file);
    if(!readConfigsBool){
        std::cout << "Error init"<< std::endl;
        return;
    }
   
    // Topics
    ros::param::get("~self_localization_pose_topic_name", self_localization_pose_topic_name);
    if ( self_localization_pose_topic_name.length() == 0)
    {
        self_localization_pose_topic_name="self_localization/pose";
    }
    std::cout<<"self_localization_pose_topic_name="<<self_localization_pose_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_pose_topic_name", motion_reference_pose_topic_name);
    if ( motion_reference_pose_topic_name.length() == 0)
    {
        motion_reference_pose_topic_name="motion_reference/pose";
    }
    std::cout<<"motion_reference_pose_topic_name="<<motion_reference_pose_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_speed_topic_name", motion_reference_speed_topic_name);
    if ( motion_reference_speed_topic_name.length() == 0)
    {
        motion_reference_speed_topic_name="motion_reference/speed";
    }
    std::cout<<"motion_reference_speed_topic_name="<<motion_reference_speed_topic_name<<std::endl;
    //
    ros::param::get("~motion_reference_path_topic_name", motion_reference_path_topic_name);
    if ( motion_reference_path_topic_name.length() == 0)
    {
        motion_reference_path_topic_name="motion_reference/path";
    }
    std::cout<<"motion_reference_path_topic_name="<<motion_reference_path_topic_name<<std::endl;
    ros::param::get("~motion_reference_remaining_path_topic_name", motion_reference_remaining_path_topic_name);
    std::cout<<"motion_reference_remaining_path_topic_name="<<motion_reference_remaining_path_topic_name<<std::endl;
}

void PathTrackerProcess::ownStart(){
    std::cout<<"START CALL"<<std::endl;
    ros::NodeHandle n;
    //Subscribers
    self_localization_pose = n.subscribe(self_localization_pose_topic_name, 1, &PathTrackerProcess::selfLocalizationPoseCallback, this);
    motion_reference_path = n.subscribe(motion_reference_path_topic_name, 1, &PathTrackerProcess::motionReferencePathCallback, this);

    // Publishers
    motion_reference_speed = n.advertise<geometry_msgs::TwistStamped>(motion_reference_speed_topic_name, 1);
    motion_reference_pose = n.advertise<geometry_msgs::PoseStamped>(motion_reference_pose_topic_name, 1);
    motion_reference_remaining_path = n.advertise<nav_msgs::Path>(motion_reference_remaining_path_topic_name, 1);
    
    //started = true;
    number_of_points = 0;
    reference_point_index = 0;
    iterate = false;
    flight_state = 0;
    end_moving_around = false;
    time = 0;
}

void PathTrackerProcess::ownStop(){
    std::cout<<"STOP CALL"<<std::endl;
    //self_localization_pose.shutdown();
    self_localization_speed.shutdown();
    motion_reference_path.shutdown();
    motion_reference_speed.shutdown();
    motion_reference_pose.shutdown();
    number_of_points = 0;
    flight_state = 0;
    reference_point_index = 0;
    iterate = false;
    end_moving_around = false;
    time = 0;
}

//This function calculates the speed when the quadrotor is moving straight
void PathTrackerProcess::speedMovingStraight(){
    //Distxy and distxyz
    distxy = sqrt(pow(reference_point.pose.position.x-current_pose.pose.position.x,2)+pow(reference_point.pose.position.y-current_pose.pose.position.y,2));
    distxyz = sqrt(pow(reference_point.pose.position.x-current_pose.pose.position.x,2)+pow(reference_point.pose.position.y-current_pose.pose.position.y,2)+pow(reference_point.pose.position.z-current_pose.pose.position.z,2));

    if (fabs(distxyz) > (precision*2) && reference_point_index != 0){
        float vectx1p = ((last_point.pose.position.x)-(reference_point.pose.position.x));
        float vecty1p = ((last_point.pose.position.y)-(reference_point.pose.position.y));
        float vectz1p = ((last_point.pose.position.z)-(reference_point.pose.position.z));
        //Calculate target points with "precision" distance
        float newpointpx = reference_point.pose.position.x + ((fabs(distxyz)-precision)*(vectx1p/(sqrt(pow(vectx1p,2)+pow(vecty1p,2)+pow(vectz1p,2)))));
        float newpointpy = reference_point.pose.position.y + ((fabs(distxyz)-precision)*(vecty1p/(sqrt(pow(vectx1p,2)+pow(vecty1p,2)+pow(vectz1p,2)))));
        float newpointpz = reference_point.pose.position.z + ((fabs(distxyz)-precision)*(vectz1p/(sqrt(pow(vectx1p,2)+pow(vecty1p,2)+pow(vectz1p,2)))));
        
        //Distances
        distx = (newpointpx-current_pose.pose.position.x);
        disty = (newpointpy-current_pose.pose.position.y);
        distz = (newpointpz-current_pose.pose.position.z);
        distxy = sqrt(pow(distx,2)+pow(disty,2));
        //Max time
        if (fabs(distxy/v_maxxy) >= fabs(distz/v_maxz)) t = fabs(precision/v_maxxy);
        else t = fabs(precision/v_maxz);
    }else{
        //Distances
        distx = (reference_point.pose.position.x-current_pose.pose.position.x);
        disty = (reference_point.pose.position.y-current_pose.pose.position.y);
        distz = (reference_point.pose.position.z-current_pose.pose.position.z);
        //Max time
        if (fabs(distxy/v_maxxy) >= fabs(distz/v_maxz)) t = fabs(distxyz/v_maxxy);
        else t = fabs(distxyz/v_maxz);
    }

    if (t != 0){
        dx = distx/t;
        dy = disty/t;
        dz = distz/t;
    }else {
        dz = 0;
        dx = 0;
        dy = 0;
    }    
}

//This function calculates the speed when the quadrotor is moving around
void PathTrackerProcess::speedMovingAround(){
    if (time == 0){
        time = ros::Time::now().toSec();
        //Previous speed : taking into account acceleration and forcing turn angle
        if (vx*next_vx <= 0) dx = dx/2;
        if (vy*next_vy <= 0) dy = dy/2;
        if (vz*next_vz <= 0) dz = dz/2;
        vx = dx;
        vy = dy;
        vz = dz;
    } 
    //Previous speed + next speed
    if ((fabs(ros::Time::now().toSec()-time) <= t_turn)&&(t_turn > 0.0)){
        dx = (((vx*(1-fabs(ros::Time::now().toSec()-time)/(t_turn))) + (next_vx*fabs(ros::Time::now().toSec()-time)/(t_turn))));
        dy = (((vy*(1-fabs(ros::Time::now().toSec()-time)/(t_turn))) + (next_vy*fabs(ros::Time::now().toSec()-time)/(t_turn))));
        dz = (((vz*(1-fabs(ros::Time::now().toSec()-time)/(t_turn))) + (next_vz*fabs(ros::Time::now().toSec()-time)/(t_turn))));
    }else{
        //Moving around ends
        end_moving_around = true;
        iterate = true;
        time = 0;
    } 
}

//This function publishes speed and yaw
void PathTrackerProcess::getOutput(){
    //Iterate if needed
    setNextReferencePoint();

    //Get next state
    nextState();
    switch(flight_state){
        case MOVING_STRAIGHT:
            speedMovingStraight();
        break;
        case MOVING_AROUND:
            speedMovingAround();
        break;
        case ENDING_MOVEMENT:
            endingMovement();
        break; 
        default:
        break;
    }
    //Publish to quadrotor pid controller
    publishValues(dx,dy,dz);
}

//This function returns the highest number
float PathTrackerProcess::max(float d1, float d2){
    if (d1 >= d2) return d1;
    else return d2;
}

//This function returns the lowest number
float PathTrackerProcess::min(float d1, float d2){
    if (d1 <= d2) return d1;
    else return d2;
}

//This function forces the vehicle to stop at the final point by reducing speed
void PathTrackerProcess::endingMovement(){
    if (time == 0){
        time = ros::Time::now().toSec();
    } 
    //Seconds to stop the vehicle
    if(fabs(ros::Time::now().toSec()-time) <= 5){
        distx = (reference_point.pose.position.x-current_pose.pose.position.x);
        disty = (reference_point.pose.position.y-current_pose.pose.position.y);
        distz = (reference_point.pose.position.z-current_pose.pose.position.z);
        distxy = sqrt(pow(reference_point.pose.position.x-current_pose.pose.position.x,2)+pow(reference_point.pose.position.y-current_pose.pose.position.y,2));
        distxyz = sqrt(pow(reference_point.pose.position.x-current_pose.pose.position.x,2)+pow(reference_point.pose.position.y-current_pose.pose.position.y,2)+pow(reference_point.pose.position.z-current_pose.pose.position.z,2));
        //Reducing speed
        if (fabs(distxy/v_maxxy) >= fabs(distz/v_maxz)) t = fabs(distxyz/(v_maxxy/2));
        else t = fabs(distxyz/(v_maxz/2));
        if (t != 0){
            dx = distx/t;
            dy = disty/t;
            dz = distz/t;
        }else {
            dz = 0; 
            dx = 0;
            dy = 0;
        }       
    }else{ //End
        number_of_points = 0;
        reference_point_index = 0;
        iterate = false;
        flight_state = 0;
        end_moving_around = false;
        time = 0;
        dx = 0;
        dy = 0;
        dz = 0;
    }
}
//This function changes the flight state
void PathTrackerProcess::nextState(){
    switch(flight_state){
        case MOVING_STRAIGHT: 
            //End trajectory
            if (reference_point_index >= number_of_points){
                flight_state = ENDING_MOVEMENT; 
            }else{
                current_distance = max (sqrt(pow(reference_point.pose.position.x-current_pose.pose.position.x,2)+pow(reference_point.pose.position.y-current_pose.pose.position.y,2)),fabs(reference_point.pose.position.z-current_pose.pose.position.z));
                //We are close to the target
                if (current_distance <= distance){
                    //If there is no turn, it continues going straight
                    if (t_turn == 0){
                        iterate = true;
                    }else{
                        flight_state = MOVING_AROUND;
                    }
                }
            }
        break;
        case MOVING_AROUND:
            //End trajectory
            if (reference_point_index >= number_of_points){
                flight_state = ENDING_MOVEMENT; 
            }else{        
                if (end_moving_around){
                    end_moving_around = false; 
                    flight_state = MOVING_STRAIGHT;
                }
            }
        break;
        case ENDING_MOVEMENT: 
        break;
        default: 
            flight_state = MOVING_STRAIGHT;
            calculateTransitionToTarget(); 
        break;
    }
}

//This function calculates turn time and next target speed
void PathTrackerProcess::calculateTransitionToTarget(){
    //Get last and current point
    if (reference_point_index == 0){
        last_point = current_pose;
    }else last_point = reference_point;
  
    reference_point = reference_path.poses[reference_point_index];
    distance = precision;
    
    //Last point
    if (reference_point_index+1 >= number_of_points){ 
        next_point = reference_point;
        t_turn = 0;
        return;
    }else next_point = reference_path.poses[reference_point_index+1];

    //Vector with direction: current point to last point
    distx = reference_point.pose.position.x-current_pose.pose.position.x;
    disty = reference_point.pose.position.y-current_pose.pose.position.y;
    distz = reference_point.pose.position.z-current_pose.pose.position.z;
    distxy = sqrt(pow(distx,2)+pow(disty,2));
    distxyz = sqrt(pow(distx,2)+pow(disty,2)+pow(distz,2));
    //Vector perpendicular to vector "dist"
    perp_vector_x1 = -disty;
    perp_vector_y1 = distx;

    //Vector with direction: current point to next point
    next_distx = next_point.pose.position.x-reference_point.pose.position.x;
    next_disty = next_point.pose.position.y-reference_point.pose.position.y;
    next_distz = next_point.pose.position.z-reference_point.pose.position.z;
    next_distxy = sqrt(pow(next_distx,2)+pow(next_disty,2));
    next_distxyz = sqrt(pow(next_distx,2)+pow(next_disty,2)+pow(next_distz,2));

    //Vector perpendicular to vector "next_dist"
    perp_vector_x2 = -next_disty;
    perp_vector_y2 = next_distx;

    //Check if the vehicle position is in the precision area
    if (min(distxyz,next_distxyz)/2<precision) distance = min(distxyz,next_distxyz)/2;

    //Get one point that is close "distance" meters to the current point
    point_to_distance_x1 = reference_point.pose.position.x + (distance*(next_distx/(sqrt(pow(next_distx,2)+pow(next_disty,2)+pow(next_distz,2)))));
    point_to_distance_y1 = reference_point.pose.position.y + (distance*(next_disty/(sqrt(pow(next_distx,2)+pow(next_disty,2)+pow(next_distz,2)))));
    //Get the other point that is close "distance" meters to the current point
    point_to_distance_x2 = reference_point.pose.position.x - (distance*(distx/(sqrt(pow(distx,2)+pow(disty,2)+pow(distz,2)))));
    point_to_distance_y2 = reference_point.pose.position.y - (distance*(disty/(sqrt(pow(distx,2)+pow(disty,2)+pow(distz,2))))); 

    if ((distxy/v_maxxy) >= fabs(distz/v_maxz)) t = (distxyz/v_maxxy);
    else t = fabs(distxyz/v_maxz);

    //Calculating current speed
    if (t != 0){
        vx = distx/t;
        vy = disty/t;
        vxy = distxy/t;
        vz = distz/t;
    }else {
        vz = 0.0;
        vx = 0.0;
        vy = 0.0;
        vxy = 0.0;
    }

    //Calculating next speed
    if ((next_distxy/v_maxxy) >= fabs(next_distz/v_maxz)){
        if (next_distxyz != 0) {
            next_vz = next_distz * v_maxxy / fabs(next_distxyz);
            next_vx = next_distx * v_maxxy / fabs(next_distxyz);
            next_vy = next_disty * v_maxxy / fabs(next_distxyz);
            next_vxy = next_distxy * v_maxxy / fabs(next_distxyz);
        }else{
            next_vz = 0.0;
            next_vx = 0.0;
            next_vy = 0.0;
            next_vxy = 0.0;
        }
    }else{
        if (next_distxyz != 0) {
            next_vx = next_distx * v_maxz / fabs(next_distxyz);
            next_vy = next_disty * v_maxz / fabs(next_distxyz);
            next_vxy = next_distxy * v_maxz / fabs(next_distxyz);
            next_vz = next_distz * v_maxz / fabs(next_distxyz);
        }else{
            next_vz = 0.0;
            next_vx = 0.0;
            next_vy = 0.0;
            next_vxy = 0.0;
        }
    }
    //Perpendicular vectors:
    //Get circumference to calculate arch length and time
    if (perp_vector_x1 == 0){ //Vector vertical
        if (perp_vector_y2 == 0){ //The other vector is horizonta
            circumference_y = point_to_distance_y2;
            circumference_x = point_to_distance_x1;
        }else{ //The other vector is not parallel to axis
            circumference_x = point_to_distance_x1;
            circumference_y = point_to_distance_y2 + (perp_vector_y2/perp_vector_x2)*(circumference_x-point_to_distance_x2);
        }
        if (perp_vector_x2 == 0){ //The other vector is also vertical
            t_turn = 0;
            return;
        }
    }else{
        if (perp_vector_y1 == 0){ // Horizontal vector
            if(perp_vector_x2 == 0){ //The other vector is vertical
                circumference_y = point_to_distance_y1;
                circumference_x = point_to_distance_x2;
            }else{// The other vector is not parallel to axis
                circumference_y = point_to_distance_y1;
                circumference_x = (perp_vector_y2*point_to_distance_x2 - perp_vector_x2*point_to_distance_y2 + perp_vector_x2*circumference_y)/perp_vector_y2;
            }
            if (perp_vector_y2 == 0){ //The other vector is also horizontal
                t_turn = 0;
                return;
            }
        }else{ // Vector is not parallel to axis
            if (perp_vector_x2 == 0){ //The other vector is vertical
                circumference_x = point_to_distance_x2;
                circumference_y = point_to_distance_y1 + (perp_vector_y1/perp_vector_x1)*(circumference_x-point_to_distance_x1); 
            }else{ //None of them are parallel to axis
                if ((perp_vector_y2/perp_vector_x2) == (perp_vector_y1/perp_vector_x1)){ //Same direction
                    t_turn = 0;
                    return;
                }else{
                    circumference_x = -(point_to_distance_y1 - point_to_distance_y2 - (perp_vector_y1*point_to_distance_x1)/perp_vector_x1 + (perp_vector_y2*point_to_distance_x2)/perp_vector_x2)/(perp_vector_y1/perp_vector_x1 - perp_vector_y2/perp_vector_x2);
                    circumference_y = point_to_distance_y1 + (perp_vector_y1/perp_vector_x1)*(circumference_x-point_to_distance_x1);
                }
            }
            if(perp_vector_y2 == 0){//The other one is horizontal
                circumference_y = point_to_distance_y2;
                circumference_x = (perp_vector_y1*point_to_distance_x1 - perp_vector_x1*point_to_distance_y1 + perp_vector_x1*circumference_y)/perp_vector_y1;
            }
        }
    }

    //Circumference's radius:
    r = fabs(sqrt(pow(point_to_distance_x2-circumference_x,2)+pow(point_to_distance_y2-circumference_y,2)));

    //Angle between both points "point_to_distance" and circumference:
    angle = fabs((atan2(circumference_y-point_to_distance_y2,circumference_x-point_to_distance_x2) * (180 / M_PI)) - (atan2 (circumference_y-point_to_distance_y1,circumference_x-point_to_distance_x1) * (180 / M_PI)));
    if(angle>180) angle = 360-angle;
    //L = 2*M_PI*r*angle / 360
    arch_lenght = (2*M_PI*r*angle) / 360;

    //Time to complete moving around
    if ((fabs(vxy)+fabs(next_vxy)) != 0) t_turn = arch_lenght/((fabs(vxy)+fabs(next_vxy))/2);
    else t_turn = 0.0;
}

//This function checks if an iteration is needed
void PathTrackerProcess::setNextReferencePoint(){
    if (iterate){
        publishRemainingPoints();
        reference_point_index++;
        iterate = false; 
        if (reference_point_index != number_of_points) calculateTransitionToTarget();
    }
}

//This function calculates and publishes the remaining points of the path
void PathTrackerProcess::publishRemainingPoints(){
    if (reference_point_index != number_of_points){
        remaining_path.poses.erase(remaining_path.poses.begin());
    }else{
        remaining_path.poses.clear();
    }
    motion_reference_remaining_path.publish(remaining_path);
}

//This function publishes speed and yaw
void PathTrackerProcess::publishValues(float dx, float dy, float dz){
    speed_result.twist.linear.x = dx;
    speed_result.twist.linear.y = dy;
    speed_result.twist.linear.z = dz;
    motion_reference_speed.publish(speed_result);
    if(path_facing){
     tf2::Quaternion orientation_quaternion;
     orientation_quaternion.setRPY(0, 0, atan2((reference_point.pose.position.y-last_point.pose.position.y),(reference_point.pose.position.x-last_point.pose.position.x)));
     reference_point.pose.orientation.w = orientation_quaternion.getW();
     reference_point.pose.orientation.x = orientation_quaternion.getX();
     reference_point.pose.orientation.y = orientation_quaternion.getY();
     reference_point.pose.orientation.z = orientation_quaternion.getZ();        
    }else{
       reference_point.pose.orientation = fixed_orientation.pose.orientation;
    }
    motion_reference_pose.publish(reference_point);
}

//This function corrects the referenced path if distance between path points are too close by getting intermediate point
void PathTrackerProcess::correctPath(){
    nav_msgs::Path reference_path_corrected;
    geometry_msgs::PoseStamped path_point;
    if(reference_path.poses.size()>1){
        while(true){
            for (int i= 0; i<reference_path.poses.size()-1; i++){
                distx = reference_path.poses[i+1].pose.position.x-reference_path.poses[i].pose.position.x;
                disty = reference_path.poses[i+1].pose.position.y-reference_path.poses[i].pose.position.y;
                distz = reference_path.poses[i+1].pose.position.z-reference_path.poses[i].pose.position.z;
                distxyz = fabs(sqrt(pow(distx,2)+pow(disty,2)+pow(distz,2)));                   
                if(distxyz < precision*2){
                    path_point.pose.position.x = (reference_path.poses[i+1].pose.position.x+reference_path.poses[i].pose.position.x)/2;
                    path_point.pose.position.y = (reference_path.poses[i+1].pose.position.y+reference_path.poses[i].pose.position.y)/2;
                    path_point.pose.position.z = (reference_path.poses[i+1].pose.position.z+reference_path.poses[i].pose.position.z)/2;
                    path_point.pose.orientation = reference_path.poses[i+1].pose.orientation;
                    reference_path_corrected.poses.push_back(path_point);
                    i++;
                }else{
                    reference_path_corrected.poses.push_back(reference_path.poses[i]);
                }

            }
            //Last point
            distx = reference_path.poses[reference_path.poses.size()-1].pose.position.x-reference_path_corrected.poses[reference_path_corrected.poses.size()-1].pose.position.x;
            disty = reference_path.poses[reference_path.poses.size()-1].pose.position.y-reference_path_corrected.poses[reference_path_corrected.poses.size()-1].pose.position.y;
            distz = reference_path.poses[reference_path.poses.size()-1].pose.position.z-reference_path_corrected.poses[reference_path_corrected.poses.size()-1].pose.position.z;
            distxyz = fabs(sqrt(pow(distx,2)+pow(disty,2)+pow(distz,2)));             
            if(distxyz <= precision*2.5){
                reference_path_corrected.poses.back()=reference_path.poses.back();
            }
            else{
                reference_path_corrected.poses.push_back(reference_path.poses[reference_path.poses.size()-1]);
            }
                if(reference_path.poses.size() == reference_path_corrected.poses.size()) return;
            
                reference_path = reference_path_corrected;
                reference_path_corrected.poses.clear();
        }
    }
}

    void PathTrackerProcess::selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_pose = *msg;
    }

    void PathTrackerProcess::motionReferencePathCallback(const nav_msgs::Path::ConstPtr& msg){
        reference_path = *msg;
        correctPath();
        remaining_path = reference_path;
        motion_reference_remaining_path.publish(remaining_path);
        number_of_points = reference_path.poses.size();
        //Initialization new path
        reference_point_index = 0;
        iterate = false;
        flight_state = 0;
        end_moving_around = false;
        time = 0;
        //Save orientation
        if(!path_facing){
            fixed_orientation = current_pose;
        }        
    }

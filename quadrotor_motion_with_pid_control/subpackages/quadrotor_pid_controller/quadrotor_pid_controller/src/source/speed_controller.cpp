/*!*******************************************************************************************
 *  \file       speed_controller.cpp
 *  \brief      Speed Controller implementation file.
 *  \details    This class is in charge of control speed in XY using PID class. 
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

#include "speed_controller.h"

//Constructor
SpeedController::SpeedController()
{
    std::cout << "Constructor: SpeedController" << std::endl;

}

//Destructor
SpeedController::~SpeedController() {}

void SpeedController::setUp(YAML::Node yamlconf)
{

    /*********************************  Speed X Controller ( from Dx to Pitch ) ************************************************/

    // Gain
    pid_dx2pitch_kp = yamlconf["speed_x"]["proportional_gain"].as<float>();
    pid_dx2pitch_kd = yamlconf["speed_x"]["derivative_gain"].as<float>();
    pid_dx2pitch_ki = yamlconf["speed_x"]["integral_gain"].as<float>();
    pid_dx2pitch_enablesat = yamlconf["speed_x"]["saturation_enabled"].as<bool>();
    pid_dx2pitch_satmin = yamlconf["speed_x"]["saturation_min"].as<float>();
    pid_dx2pitch_satmax = yamlconf["speed_x"]["saturation_max"].as<float>();
    pid_dx2pitch_enableantiwp = yamlconf["speed_x"]["anti_wind_up_enabled"].as<bool>();
    pid_dx2pitch_kw = yamlconf["speed_x"]["anti_wind_up"].as<float>();


    /*********************************  Speed Y Controller ( from Dy to Roll ) **************************************************/

    // Gain
    pid_dy2roll_kp = yamlconf["speed_y"]["proportional_gain"].as<float>();
    pid_dy2roll_kd = yamlconf["speed_y"]["derivative_gain"].as<float>();
    pid_dy2roll_ki = yamlconf["speed_y"]["integral_gain"].as<float>();
    pid_dy2roll_enablesat = yamlconf["speed_y"]["saturation_enabled"].as<bool>();
    pid_dy2roll_satmin = yamlconf["speed_y"]["saturation_min"].as<float>();
    pid_dy2roll_satmax = yamlconf["speed_y"]["saturation_max"].as<float>();
    pid_dy2roll_enableantiwp = yamlconf["speed_y"]["anti_wind_up_enabled"].as<bool>();
    pid_dy2roll_kw = yamlconf["speed_y"]["anti_wind_up"].as<float>();


    /*********************************  Max Speed Reference *************************************/
    pid_speed_enablemax= yamlconf["speed_y"]["vxy_max_enabled"].as<bool>();
    pid_speed_max= yamlconf["speed_y"]["vxy_max"].as<float>();

    std::cout << "Constructor: SpeedController...Exit" << std::endl;

}

void SpeedController::start()
{

    // Reset PID
    PID_Dx2Pitch.reset();
    PID_Dy2Roll.reset();

}


void SpeedController::stop()
{

}


void SpeedController::setFeedback(float velx, float vely){
    PID_Dx2Pitch.setFeedback(velx);
    PID_Dy2Roll.setFeedback(vely);
}
void SpeedController::setReference(float ref_velx, float ref_vely){

    if (pid_speed_enablemax){
        float vmax=sqrt(pow(ref_velx,2)+pow(ref_vely,2));
        if (vmax > pid_speed_max){
            ref_velx=ref_velx/vmax*pid_speed_max;
            ref_vely=ref_vely/vmax*pid_speed_max;
        }
    }

    PID_Dx2Pitch.setReference(ref_velx);
    PID_Dy2Roll.setReference(ref_vely);
}

void SpeedController::getOutput(float *pitchb, float *rollb, float yaw){

    /*********************************  Speed X Controller ( from Dx to Pitch ) ************************************************/

    PID_Dx2Pitch.setGains(pid_dx2pitch_kp,pid_dx2pitch_ki,pid_dx2pitch_kd);
    PID_Dx2Pitch.enableMaxOutput(pid_dx2pitch_enablesat,pid_dx2pitch_satmin,pid_dx2pitch_satmax);
    PID_Dx2Pitch.enableAntiWindup(pid_dx2pitch_enableantiwp,pid_dx2pitch_kw);

    pitch = PID_Dx2Pitch.getOutput();


    /*********************************  Speed Y Controller ( from Dy to Roll ) **************************************************/

    PID_Dy2Roll.setGains(pid_dy2roll_kp,pid_dy2roll_ki,pid_dy2roll_kd);
    PID_Dy2Roll.enableMaxOutput(pid_dy2roll_enablesat,pid_dy2roll_satmin,pid_dy2roll_satmax);
    PID_Dy2Roll.enableAntiWindup(pid_dy2roll_enableantiwp,pid_dy2roll_kw);


    roll = PID_Dy2Roll.getOutput();


    /******************************** World to Body Transformation ********************************************/

    *pitchb =  pitch * cos (yaw)   + roll * sin (yaw);
    *rollb  =  + pitch * sin (yaw) - roll * cos (yaw);

}


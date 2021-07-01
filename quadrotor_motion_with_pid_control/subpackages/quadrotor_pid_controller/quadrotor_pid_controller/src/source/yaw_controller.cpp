/*!*******************************************************************************************
 *  \file       yaw_controller.cpp
 *  \brief      Yaw controller implementation file.
 *  \details    This class is in charge of control yaw angle using PID class. 
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


#include "yaw_controller.h"

//Constructor
YawController::YawController()
{

    std::cout << "Constructor: YawController" << std::endl;

}

//Destructor
YawController::~YawController() {}

void YawController::setUp(YAML::Node yamlconf)
{

    /*********************************   Yaw Controller ( from Yaw to DYaw ) ************************************************/

    // Gain
    pid_yaw2dyaw_kp = yamlconf["yaw"]["proportional_gain"].as<float>();
    pid_yaw2dyaw_kd = yamlconf["yaw"]["derivative_gain"].as<float>();
    pid_yaw2dyaw_ki = yamlconf["yaw"]["integral_gain"].as<float>();
    pid_yaw2dyaw_enablesat = yamlconf["yaw"]["saturation_enabled"].as<bool>();
    pid_yaw2dyaw_satmin = yamlconf["yaw"]["saturation_min"].as<float>();
    pid_yaw2dyaw_satmax = yamlconf["yaw"]["saturation_max"].as<float>();
    pid_yaw2dyaw_enableantiwp = yamlconf["yaw"]["anti_wind_up_enabled"].as<bool>();
    pid_yaw2dyaw_kw = yamlconf["yaw"]["anti_wind_up"].as<float>();

    std::cout << "Constructor: YawController...Exit" << std::endl;
}

void YawController::start()
{

    // Reset PID
    PID_Yaw2DYaw.reset();

}

void YawController::stop()
{

}

void YawController::setFeedback(float yaw ){

    PID_Yaw2DYaw.setFeedback(yaw);

}

void YawController::setReference(float ref_yaw, float yaw_aux){

    error_yaw = ref_yaw - yaw_aux;

    if (error_yaw > M_PI){
        ref_yaw -= 2 * M_PI;
    }
    else if(error_yaw < -M_PI){
        ref_yaw += 2 * M_PI;
    }

    PID_Yaw2DYaw.setReference(ref_yaw);

}

void YawController::getOutput(float *dyaw){

    /*********************************  Yaw Controller ( from Yaw to dYaw ) ************************************************/

    PID_Yaw2DYaw.setGains(pid_yaw2dyaw_kp,pid_yaw2dyaw_ki,pid_yaw2dyaw_kd);
    PID_Yaw2DYaw.enableMaxOutput(pid_yaw2dyaw_enablesat,pid_yaw2dyaw_satmin,pid_yaw2dyaw_satmax);
    PID_Yaw2DYaw.enableAntiWindup(pid_yaw2dyaw_enableantiwp,pid_yaw2dyaw_kw);

    *dyaw = PID_Yaw2DYaw.getOutput();

}

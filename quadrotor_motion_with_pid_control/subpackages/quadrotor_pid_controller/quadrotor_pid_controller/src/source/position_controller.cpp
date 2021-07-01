/*!*******************************************************************************************
 *  \file       position_controller.cpp
 *  \brief      Position Controller implementation file.
 *  \details    This class is in charge of control position XY using PID class.
 *              Sends as Outputs speed references.
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

#include "position_controller.h"

//Constructor
PositionController::PositionController()
{
    std::cout << "Constructor: PositionController" << std::endl;

}

//Destructor
PositionController::~PositionController() {}

void PositionController::setUp(YAML::Node yamlconf)
{


    /*********************************  Position X Controller ( from X to Dx ) ************************************************/

    // Gain
    pid_x2dx_kp = yamlconf["position_x"]["proportional_gain"].as<float>();
    pid_x2dx_kd = yamlconf["position_x"]["derivative_gain"].as<float>();    
    pid_x2dx_ki = yamlconf["position_x"]["integral_gain"].as<float>();
    pid_x2dx_enablesat = yamlconf["position_x"]["saturation_enabled"].as<bool>();
    pid_x2dx_satmin = yamlconf["position_x"]["saturation_min"].as<float>();
    pid_x2dx_satmax = yamlconf["position_x"]["saturation_max"].as<float>();
    pid_x2dx_enableantiwp = yamlconf["position_x"]["anti_wind_up_enabled"].as<bool>();
    pid_x2dx_kw = yamlconf["position_x"]["anti_wind_up"].as<float>();


    /*********************************  Position Y Controller ( from Y to Dy ) **************************************************/

    // Gain
    pid_y2dy_kp = yamlconf["position_y"]["proportional_gain"].as<float>();
    pid_y2dy_kd = yamlconf["position_y"]["derivative_gain"].as<float>();
    pid_y2dy_ki = yamlconf["position_y"]["integral_gain"].as<float>();
    pid_y2dy_enablesat = yamlconf["position_y"]["saturation_enabled"].as<bool>();
    pid_y2dy_satmin = yamlconf["position_y"]["saturation_min"].as<float>();
    pid_y2dy_satmax = yamlconf["position_y"]["saturation_max"].as<float>();
    pid_y2dy_enableantiwp = yamlconf["position_y"]["anti_wind_up_enabled"].as<bool>();
    pid_y2dy_kw = yamlconf["position_y"]["anti_wind_up"].as<float>();

    std::cout << "Constructor: PositionController...Exit" << std::endl;


}

void PositionController::start()
{

    // Reset PID
    PID_X2Dx.reset();
    PID_Y2Dy.reset();

}

void PositionController::stop()
{

}


void PositionController::setFeedback(float posx, float posy ){
    PID_X2Dx.setFeedback(posx);
    PID_Y2Dy.setFeedback(posy);
}
void PositionController::setReference(float ref_posx, float ref_posy){
    PID_X2Dx.setReference(ref_posx);
    PID_Y2Dy.setReference(ref_posy);

}

void PositionController::getOutput(float *dx, float *dy){

    /************************ Position X Controller ( from X to Dx )   *****************************/

    PID_X2Dx.setGains(pid_x2dx_kp,pid_x2dx_ki,pid_x2dx_kd);
    PID_X2Dx.enableMaxOutput(pid_x2dx_enablesat,pid_x2dx_satmin,pid_x2dx_satmax);
    PID_X2Dx.enableAntiWindup(pid_x2dx_enableantiwp,pid_x2dx_kw);

    *dx = PID_X2Dx.getOutput();


    /************************ Position Y Controller ( from Y to Dy ) *****************************/

    PID_Y2Dy.setGains(pid_y2dy_kp,pid_y2dy_ki,pid_y2dy_kd);
    PID_Y2Dy.enableMaxOutput(pid_y2dy_enablesat,pid_y2dy_satmin,pid_y2dy_satmax);
    PID_Y2Dy.enableAntiWindup(pid_y2dy_enableantiwp,pid_y2dy_kw);

    *dy = PID_Y2Dy.getOutput();
}

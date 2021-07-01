/*!*******************************************************************************************
 *  \file       altitude_controller.cpp
 *  \brief      Altitude controller implementation file.
 *  \details    This class is in charge of control altitude level using PID class. 
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

#include "altitude_controller.h"

//Constructor
AltitudeController::AltitudeController()
{
    std::cout << "Constructor: AltitudeController" << std::endl;

}

//Destructor
AltitudeController::~AltitudeController() {}

void AltitudeController::setUp(YAML::Node yamlconf)
{

    pid_z2dz_kp = yamlconf["position_z"]["proportional_gain"].as<float>();
    pid_z2dz_kd = yamlconf["position_z"]["derivative_gain"].as<float>();
    pid_z2dz_ki = yamlconf["position_z"]["integral_gain"].as<float>();
    pid_z2dz_enablesat = yamlconf["position_z"]["saturation_enabled"].as<bool>();
    pid_z2dz_satmin = yamlconf["position_z"]["saturation_min"].as<float>();
    pid_z2dz_satmax = yamlconf["position_z"]["saturation_max"].as<float>();
    pid_z2dz_enableantiwp = yamlconf["position_z"]["anti_wind_up_enabled"].as<bool>();
    pid_z2dz_kw = yamlconf["position_z"]["anti_wind_up"].as<float>();
    pid_z2dz_enableff = yamlconf["position_z"]["feedforward_enabled"].as<bool>();
    pid_z2dz_ffmass = yamlconf["position_z"]["feedforward_mass"].as<float>();
    pid_z2dz_fffactor = yamlconf["position_z"]["feedforward_factor"].as<float>();

    std::cout << "Constructor: AltitudeController...Exit" << std::endl;

}

void AltitudeController::start()
{

    // Reset PID
    PID_Z2Dz.reset();

}

void AltitudeController::stop()
{

}

void AltitudeController::setFeedback(float height ){

    PID_Z2Dz.setFeedback(height);

}
void AltitudeController::setReference(float ref_h){

    PID_Z2Dz.setReference(ref_h);

}

void AltitudeController::getOutput(float *dh){

    /*********************************  Z Controller ( from Z to dZ ) ************************************************/

    PID_Z2Dz.setGains(pid_z2dz_kp,pid_z2dz_ki,pid_z2dz_kd);
    PID_Z2Dz.enableMaxOutput(pid_z2dz_enablesat,pid_z2dz_satmin,pid_z2dz_satmax);
    PID_Z2Dz.enableAntiWindup(pid_z2dz_enableantiwp,pid_z2dz_kw);
    PID_Z2Dz.enableFeedForward(pid_z2dz_enableff,pid_z2dz_ffmass,pid_z2dz_fffactor);

    *dh = PID_Z2Dz.getOutput();
}


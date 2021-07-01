/*!*******************************************************************************************
 *  \file       yaw_controller.h
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

#ifndef yaw_controller
#define yaw_controller

#include "pid_control.h"

#include <yaml-cpp/yaml.h>
#include "xmlfilereader.h"
#include <math.h>

class YawController
{
public:

  void setUp(YAML::Node yamlconf);

  void start();

  void stop();

  void getOutput(float *dyaw);
  void setReference(float ref_yaw, float yaw_aux);
  void setFeedback(float yaw);

  //! Constructor. \details Same arguments as the ros::init function.
  YawController();

  //! Destructor.
  ~YawController();

private:

  PID PID_Yaw2DYaw;

  float error_yaw;

  // Controller Tuning Parameters

  float pid_yaw2dyaw_kp;
  float pid_yaw2dyaw_ki;
  float pid_yaw2dyaw_kd;
  bool pid_yaw2dyaw_enablesat;
  float pid_yaw2dyaw_satmax;
  float pid_yaw2dyaw_satmin;
  bool pid_yaw2dyaw_enableantiwp;
  float pid_yaw2dyaw_kw;

};
#endif

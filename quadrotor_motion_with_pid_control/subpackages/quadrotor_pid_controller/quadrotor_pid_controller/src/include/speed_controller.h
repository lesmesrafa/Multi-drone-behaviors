/*!*******************************************************************************************
 *  \file       speed_controller.h
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

#ifndef speed_controller
#define speed_controller

#include "pid_control.h"

#include "xmlfilereader.h"
#include <yaml-cpp/yaml.h>
#include <math.h>

class SpeedController
{
public:

  void setUp(YAML::Node yamlconf);

  void start();

  void stop();

  void getOutput(float *pitchb, float *rollb, float yaw);
  void setReference(float ref_velx, float ref_vely);
  void setFeedback(float velx, float vely);

  //! Constructor. \details Same arguments as the ros::init function.
  SpeedController();

  //! Destructor.
  ~SpeedController();

private:

  PID PID_Dx2Pitch;
  PID PID_Dy2Roll;

  // Output
  float pitch;
  float roll;

  // Gains

  float pid_dx2pitch_kp;
  float pid_dx2pitch_ki;
  float pid_dx2pitch_kd;
  bool pid_dx2pitch_enablesat;
  float pid_dx2pitch_satmax;
  float pid_dx2pitch_satmin;
  bool pid_dx2pitch_enableantiwp;
  float pid_dx2pitch_kw;


  float pid_dy2roll_kp;
  float pid_dy2roll_ki;
  float pid_dy2roll_kd;
  bool pid_dy2roll_enablesat;
  float pid_dy2roll_satmax;
  float pid_dy2roll_satmin;
  bool pid_dy2roll_enableantiwp;
  float pid_dy2roll_kw;


  float pid_speed_max;
  bool pid_speed_enablemax;

};
#endif

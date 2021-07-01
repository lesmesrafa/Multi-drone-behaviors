/*!*******************************************************************************************
 *  \file       pid_control.cpp
 *  \brief      PID Control implementation file.
 *  \details    This class defines the controller PID.
 *  \authors    Alberto Rodelgo Perales
 *              Pablo Santofimia Ruiz
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
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

#include <iostream>
#include "pid_control.h"

PID::PID() {
    kp = 0.0;
    ki = 0.0;
    kd = 0.0;
    fl = 0.2;
    feedback = 0.0;
    reference = 0.0;
    output = 0.0;
    integrator = 0.0;
    last_error = 0.0;
    average = 0.0;
    started = false;
    feedforward_enabled=false;
    feedforward_value=0.0;
    enableAntiWindup(false, 0.0);
    enableMaxOutput(false, 0.0);
    enableFeedForward(false, 0.0, 0.0);

}

PID::~PID() {
}

float PID::getOutput() {
    elapsed = timer.getElapsedSeconds();
	timer.restart(started);
	if (!started) {
		started = true;
		return output;
	}

	// Calculate error and derivative
    error = reference - feedback;
    float derror = 0;
    if (elapsed != 0) derror= (error - last_error) / elapsed;
    last_error = error;
    average = derror*fl + (1-fl)*average;   //! Derivative Filter

	// Output
    output = kp * error + ki * integrator + kd * average;

    if (feedforward_enabled) {
        output += feedforward_value;
    }

    // Anti-windup
    if (antiwindup_enabled && saturation_enabled && ki != 0) {

        float outputpresat = output;
        if (output > saturation_max) output = saturation_max;
        if (output < saturation_min) output = saturation_min;
        integrator += kw * (output - outputpresat) / ki + error * elapsed;

	} else integrator += error * elapsed;

    // Saturation
    if (saturation_enabled && !antiwindup_enabled) {
        if (output > saturation_max) output = saturation_max;
        if (output < saturation_min) output = saturation_min;
	}

	return output;
}

void PID::enableAntiWindup(bool enable, float w) {

    antiwindup_enabled = enable;
    kw = w;
}

void PID::enableMaxOutput(bool enable, float max) {
    enableMaxOutput( enable, -max, +max);
}

void PID::enableMaxOutput(bool enable, float min, float max) {
    saturation_enabled = enable;
    saturation_min = min;
    saturation_max = max;
}

void PID::enableFeedForward(bool enable, float mass, float fffactor){
    feedforward_enabled = enable;
    feedforward_value = mass*fffactor;
}

void PID::reset() {
    integrator = 0.0;
    last_error = 0.0;
    average = 0.0;
    started = false;
}

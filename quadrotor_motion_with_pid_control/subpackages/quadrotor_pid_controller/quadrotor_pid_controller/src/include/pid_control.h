/*!*******************************************************************************************
 *  \file       pid_control.h
 *  \brief      PID Control implementation file.
 *  \details    This class defines the controller PID.
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



#ifndef pid_control
#define pid_control

#include "Timer.h"
#include <math.h>


class PID {
private:
    // Input/output
    float reference;
    float feedback;
    float output;

    // Parameters
    float kp, ki, kd, kw;
    float fl;
    bool  antiwindup_enabled, saturation_enabled, feedforward_enabled;
    float antiwindup_min, antiwindup_max;
    float saturation_min, saturation_max;
    float feedforward_value;

    // Internal state
    float integrator;
    float last_error;
    float error;
    float average;
    float elapsed;
    Timer timer;
    bool started;

public:
    PID();
    ~PID();

    inline void setGains(const float p, const float i, const float d) { kp = p; ki = i; kd = d; }
    inline void setDerFactor(const float d){fl = d;}
    inline void getGains(float &p, float &i, float &d) { p = kp; i = ki; d = kd; }
    inline void setInt(float integ) { integrator = integ; }
    inline float getInt() { return integrator; }
    inline float getError() {return error = reference - feedback; }
    void enableMaxOutput(bool enable, float max);
    void enableAntiWindup(bool enable, const float w);
    void enableMaxOutput(bool enable, float min, float max);
    void enableFeedForward(bool enable, float mass, float fffactor);

    inline void setReference(float ref) { reference = ref; }
    inline void setFeedback(float measure) { feedback = measure; }
    float getOutput();

    void reset();
};
#endif /* pid_control */

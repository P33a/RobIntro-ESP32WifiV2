/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "robot.h"
#include "IRLine.h"

robot_t::robot_t()
{
  pulses_to_meters = 0.0000532;
  wheel_dist = 0.13;
  dv_max = 5;
  dw_max = 10;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  v1e = pulses_to_meters * enc1 / dt;
  v2e = pulses_to_meters * enc2 / dt;
  
  // Alt version
  //  v1e = TWO_PI * r1 / (2 * 1920.0 * dt) * enc1;
  //  v2e = TWO_PI * r2 / (2 * 1920.0 * dt) * enc2;

  // Estimate robot speed
  ve = (v1e + v2e) / 2;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  x += ds * cos(theta + dtheta/2);
  y += ds * sin(theta + dtheta/2);
  theta = theta + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}


void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v = Vnom;
  w = Wnom;
}


void robot_t::followLineRight(IRLine_t& IRLine, float Vnom, float K)
{
  v = Vnom;
  w = K * IRLine.pos_right;
}


void robot_t::followLineLeft(IRLine_t& IRLine, float Vnom, float K)
{
  v = Vnom;
  w = K * IRLine.pos_left;
}

void robot_t::followLine(IRLine_t& IRLine, float Vnom, float K)
{
  float pos;

  if (fabs(IRLine.pos_left) < fabs(IRLine.pos_right)) {
    pos = IRLine.pos_left;
  } else {
    pos = IRLine.pos_right;
  }

  v = Vnom;
  w = K * pos;
}



void robot_t::setState(byte new_state)
{
  tes = millis();
  tis = 0;
  state = new_state;
}

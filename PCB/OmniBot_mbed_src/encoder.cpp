/*The MIT License (MIT)

Copyright (c) 2015 Zijian Wang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include "mbed.h"
#include "encoder.h"
#include "QEI.h" // encoder library
#include "config.h"


// external variables
float vel_wheel[4]; // linear velocity of each wheel, unit: m/s
float vel_wheel_filter[4]; // outcome of a low pass filter
extern Serial pc; // for debug


// internal stuff
QEI enc1(D7,D8,NC,1920, QEI::X4_ENCODING);
QEI enc2(PC_1,PC_0,NC,1920, QEI::X4_ENCODING);
QEI enc3(D13,D12,NC,1920, QEI::X4_ENCODING);
QEI enc4(PB_7,PC_10,NC,1920, QEI::X4_ENCODING);

Timer timer_enc;

int pulse[4], pulse_old[4]; // number of pulses read back from encoders
float vel_wheel_pre[4]; // used for filtering
float delta_t;

void Encoder_Read(void) {
    // read encoders
    pulse[0] = enc1.getPulses();
    pulse[1] = enc2.getPulses();
    pulse[2] = enc3.getPulses();
    pulse[3] = enc4.getPulses();
    // get time interval from timer
    delta_t = timer_enc.read_us()/1000000.0;
    // reset timer
    timer_enc.reset();
    // calculate wheel velocity (linear)
    vel_wheel[0] = (float)((pulse[0] - pulse_old[0])) / 1920.0 * 6.28318 / delta_t * 0.03; // 0.03(m) is the radius of the wheel
    vel_wheel[1] = (float)((pulse[1] - pulse_old[1])) / 1920.0 * 6.28318 / delta_t * 0.03;
    vel_wheel[2] = (float)((pulse[2] - pulse_old[2])) / 1920.0 * 6.28318 / delta_t * 0.03;
    vel_wheel[3] = (float)((pulse[3] - pulse_old[3])) / 1920.0 * 6.28318 / delta_t * 0.03;
    // update old pulse counts
    pulse_old[0] = pulse[0];
    pulse_old[1] = pulse[1];
    pulse_old[2] = pulse[2];
    pulse_old[3] = pulse[3];
    // low pass filter on wheel velocity
    for(int i=0; i<4; i++) {
        vel_wheel_filter[i] = 0.5*vel_wheel_pre[i] + 0.5*vel_wheel[i]; // no need to implement a strong low-pass filter, because using encoder, we already have the average speed
        vel_wheel_pre[i] = vel_wheel[i];         
    }
#ifdef PRINT_WHEEL_SPEED
    pc.printf("[Wheel Spd] %.3f, %.3f, %.3f, %.3f\n",vel_wheel_filter[0],vel_wheel_filter[1],vel_wheel_filter[2],vel_wheel_filter[3]);        
#endif
}

void Encoder_Init(void) {
    timer_enc.start();        
}




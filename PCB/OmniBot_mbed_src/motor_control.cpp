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
#include "motor_control.h"
#include "motordriver.h" // H-bridge library
#include "math.h"

// Extern variables
extern float vel_wheel_filter[4]; // velocity of the motors, from encoders

// Internal variables

Motor motor1(PB_4,PA_15,0);  //(pwm_pin, dir_pin, breakable)
Motor motor2(PB_5,PC_11,0);
Motor motor3(PC_8,PC_12,0);
Motor motor4(PC_9,PD_2,0);

float motor_set_speed[4];

void Motor_Set_Speed(unsigned char _motor, float _vel) { // _motor:{1,2,3,4}, _vel:[-1,1]
    motor_set_speed[_motor-1] = _vel;   
}

float sum_err[4];
void Motor_PID(void) {
    float err=0;
    //=========== Motor 1 ===========
    err = motor_set_speed[0] - vel_wheel_filter[0];         
    sum_err[0] += err;
    if( abs(motor_set_speed[0])<0.02 && abs(vel_wheel_filter[0])<0.03 )
        motor1.stop(); // protect the motor
    else
        motor1.speed(motor_set_speed[0]*KFW + KP*err + KI*sum_err[0] ); // feed forward + PI     
    // =========== Motor 2 ===========
    err = motor_set_speed[1] - vel_wheel_filter[1];         
    sum_err[1] += err;
    if( abs(motor_set_speed[1])<0.02 && abs(vel_wheel_filter[1])<0.03 )
        motor2.stop(); // protect the motor
    else
        motor2.speed(motor_set_speed[1]*KFW + KP*err + KI*sum_err[1] ); // feed forward + PI  
    // =========== Motor 3 ===========         
    err = motor_set_speed[2] - vel_wheel_filter[2];         
    sum_err[2] += err;
    if( abs(motor_set_speed[2])<0.02 && abs(vel_wheel_filter[2])<0.03 )
        motor3.stop(); // protect the motor
    else
        motor3.speed(motor_set_speed[2]*KFW + KP*err + KI*sum_err[2] ); // feed forward + PI    
    // =========== Motor 4 ===========         
    err = motor_set_speed[3] - vel_wheel_filter[3];         
    sum_err[3] += err;
    if( abs(motor_set_speed[3])<0.02 && abs(vel_wheel_filter[3])<0.03 )
        motor4.stop(); // protect the motor
    else
        motor4.speed(motor_set_speed[3]*KFW + KP*err + KI*sum_err[3] ); // feed forward + PI  
}


void Motor_Stop_All(void) {
    for(int i=0; i<4; i++) {
        motor_set_speed[i] = 0;    
    }
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();    
}


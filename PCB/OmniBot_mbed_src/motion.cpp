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
#include "math.h"
#include "motion.h"
#include "motor_control.h"
#include "config.h"

void Robot_Set_Motion(float vel_mag, float vel_ang, float omega) {
#ifdef LIMIT_SPD    
    if(vel_mag > 0.7) vel_mag = 0.7;
    if(omega > 0.4) omega = 0.4;    
    if(omega < -0.4) omega = -0.4;
#endif    
    
    float set_angle_rad = (vel_ang-45)*3.14159/180.0;
    float v_24 = vel_mag * sin(set_angle_rad);
    float v_13 = vel_mag * cos(set_angle_rad);
    
    Motor_Set_Speed(1, v_13  - omega);        
    Motor_Set_Speed(2, -v_24 - omega);      
    Motor_Set_Speed(3, -v_13 - omega);  
    Motor_Set_Speed(4, v_24  - omega);  
}
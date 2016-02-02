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
#include "motordriver.h"
#include "comm.h"
#include "encoder.h"
#include "motor_control.h"
#include "motion.h"
#include "adc.h"
#include "config.h" // configuration of the robot, debug info

// Update from previous version
// 1. Communication protocol now transmit HEX values

// TODO: 
// 1. Carefully examine the watchdog program. Figure out why the program goes dead
// 2. The robot speed!
// 3. Communication (HEX)!
// 4. Odometry

Ticker tick_led;
DigitalOut myled(PB_2);
Serial pc(USBTX,USBRX);
Timer timer_main;

int watch_dog = 0;

extern Motor motor1;  //(pwm_pin, dir_pin, breakable)
extern Motor motor2;
extern Motor motor3;
extern Motor motor4;


// command from rpi
extern float vel_mag; 
extern float vel_dir;
extern float omega;

void flip_led() {
    myled = !myled;
    
    // the following is used as watchdog
    if(watch_dog != 0) { // system hangs. Reset the MCU
        NVIC_SystemReset();
    } else {
        watch_dog = 1;
    }
}

int main() {
    pc.baud(115200);
    myled = 0;
    
    Init_Ser_RPi();
    Encoder_Init();
    
    Motor_Stop_All();
    Current_Sensor_Calib();   
    
    timer_main.start();
    
    
    watch_dog = 1;
    tick_led.attach(&flip_led, 0.5);
    while(1) {
        //pc.printf("%.2f,%.2f,%.2f,%.2f\n", vel_mag, vel_ang, omega, omega_dir);
        //wait(0.1);

        //if(vel_ang<180)
        //    Motor_Set_Speed(3,vel_mag/600.0);
        //else
        //    Motor_Set_Speed(3,-vel_mag/600.0);
        //motor1.stop();
        
        //pc.printf("vel_mag = %f, vel_dir = %f, omega = %f\n", vel_mag, vel_dir, omega);
        Robot_Set_Motion(vel_mag/600.0, vel_dir, omega/600.0);
    
        
        // The following functions must be included in the main loop
        Encoder_Read();
        Motor_PID();
        Current_Read();
        
        // Clear WatchDog
        watch_dog = 0;
        
        //wait(0.001);
        //pc.printf("[Loop: %d ms]  ", timer_main.read_ms());
        timer_main.reset();
    }
}

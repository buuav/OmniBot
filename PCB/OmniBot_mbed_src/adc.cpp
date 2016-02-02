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
#include "adc.h"
#include "config.h"
#include "motor_control.h"

// extern stuff
float cur_sen_voltage_filter[4]; 
float cur_sen_offset[4] = {2.53,2.53,2.53,2.53}; // get from calibration, intial value is 2.53 volt
float current[4];

extern Serial pc;

// intern stuff
AnalogIn Cur_Sen1(PC_2);
AnalogIn Cur_Sen2(PC_3);
AnalogIn Cur_Sen3(PC_4);
AnalogIn Cur_Sen4(PC_5);

float cur_sen_voltage[4]; // raw voltage of current sensors
void Current_Read(void) {
    float v_sum;
    // read the voltage from current sensor
    v_sum = 0; // for sensor 1
    for(int j=0; j<N_SAMPLE; j++) {
        v_sum += Cur_Sen1.read();                
    }
    cur_sen_voltage[0] = v_sum / N_SAMPLE * 3.3;    
    v_sum = 0; // for sensor 2
    for(int j=0; j<N_SAMPLE; j++) {
        v_sum += Cur_Sen2.read();                
    }
    cur_sen_voltage[1] = v_sum / N_SAMPLE * 3.3; 
    v_sum = 0; // for sensor 3
    for(int j=0; j<N_SAMPLE; j++) {
        v_sum += Cur_Sen3.read();                
    }
    cur_sen_voltage[2] = v_sum / N_SAMPLE * 3.3; 
    v_sum = 0; // for sensor 4
    for(int j=0; j<N_SAMPLE; j++) {
        v_sum += Cur_Sen4.read();                
    }
    cur_sen_voltage[3] = v_sum / N_SAMPLE * 3.3;   
    
    // low pass filter on current sensor output, calculate current value
    for(int i=0; i<4; i++) {
        cur_sen_voltage_filter[i] = 0.95*cur_sen_voltage_filter[i] + 0.05*cur_sen_voltage[i];
        current[i] = (cur_sen_voltage_filter[i] - cur_sen_offset[i])/0.185;
    }
    //pc.printf("%.3f,%.3f,%.3f,%.3f\n", cur_sen_voltage_filter[0], cur_sen_voltage_filter[1], cur_sen_voltage_filter[2], cur_sen_voltage_filter[3] );
#ifdef PRINT_MOTOR_CURRENT    
    pc.printf("[Cur] % 05.3f, % 05.3f, % 05.3f, % 05.3f\n", current[0], current[1], current[2], current[3]); 
#endif  
}

void Current_Sensor_Calib(void) {
    pc.printf("\nStart calibrating current sensors......\n");
    
    Motor_Stop_All();
    wait(1);
    for(int k=0; k<500; k++) Current_Read(); // let readings reach equilibrium        
    
    float temp[4] = {0,0,0,0};
    for(int i=0; i<20; i++) { // do 20 measurements and take the average
        Current_Read();
        for(int j=0; j<4; j++) {
            temp[j] += cur_sen_voltage_filter[j];        
        }            
    }
    for(int j=0; j<4; j++) {
        cur_sen_offset[j] = temp[j] / 20.0;    
    }   
    pc.printf("Offset voltages: %.3fV, %.3fV, %.3fV, %.3fV\n", cur_sen_offset[0], cur_sen_offset[1], cur_sen_offset[2], cur_sen_offset[3]);
    pc.printf("Current sensor calibration completed!\n");
}



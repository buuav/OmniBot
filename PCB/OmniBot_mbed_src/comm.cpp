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
#include "comm.h"
#include "math.h"

extern Serial pc;
Serial rpi(PB_6,PA_10); //set Tx, Rx pins

    
float vel_mag; 
float vel_dir;
float omega;


typedef union {
    float f;
    unsigned char ch[4];
} float_union;

float hex2float(char *addr) {
    float_union fu;
    fu.ch[0] = addr[0];   
    fu.ch[1] = addr[1]; 
    fu.ch[2] = addr[2]; 
    fu.ch[3] = addr[3];
    
    return fu.f;
}


char rec[CMD_BUFFER_SIZE];
int rec_count;  
int rec_length = 15;

void execute(char * cmd) {
    // check sum
    unsigned char checksum = 0;
    for(int i=1; i<=rec_length-2; i++) { // checksum will exclude the first byte ('@') and the last byte (checksum byte)
        checksum += cmd[i];       
    }
    if(checksum == cmd[rec_length-1]) {
        // take action when receiving message correctly
        switch(cmd[1]) {
            case CMD_MOTION_CONTROL:  // 'a'
                vel_mag = hex2float(&cmd[2]);
                vel_dir = hex2float(&cmd[6]);
                omega = hex2float(&cmd[10]);      
                //pc.printf("vel_mag = %f, vel_dir = %f, omega = %f\n", vel_mag, vel_dir, omega);  
                break;
            case CMD_PLAIN: // 'z', test info
                //pc.printf("%s\n", cmd);
                break;
            default: 
                // TODO: unrecongnized command
                break;
        }
    } else {
        // TODO: checksum unmatched        
    }    
          
    // pc.printf("Expected checksum = %d, actual = %d\n", cmd[rec_length-1], checksum);
}

void rpi_read(void){
    if (rpi.readable()) { // record the incoming byte
        rec[rec_count] = rpi.getc();  
        rec_count++;   
    }
    if (rec_count == 1 && rec[0]!='@') { // capture start byte
        rec_count = 0;
        rec_length = 15; 
    }
    if (rec_count == 2) { // determine command length ro receive
        switch(rec[1]) {
            case CMD_MOTION_CONTROL:
                rec_length = CMD_MOTION_CONTROL_SIZE;
                break;
            case CMD_PLAIN:
                rec_length = CMD_PLAIN_SIZE;
                break;
            default:
                // TODO: unrecognized command type
            
                break;
        }      
    }
    if (rec_count >= rec_length) {
       execute(rec); 
       rec_count = 0;
       rec_length = 15;
    }
}

void Init_Ser_RPi(void)
{
    rpi.baud(115200);
    rpi.attach(&rpi_read);
}

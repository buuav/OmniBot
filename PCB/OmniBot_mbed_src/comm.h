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

#ifndef COMM_H
#define COMM_H
//Communication with Raspberry Pi

#define CMD_BUFFER_SIZE 20
#define CMD_MOTION_CONTROL 'a' // format: "@a + vel_mag(4-byte-float) + vel_dir(4-byte-float) + omega(4-byte-float) + checksum(1-byte)"
#define CMD_MOTION_CONTROL_SIZE 15
#define CMD_PLAIN 'z' // ASCII msg. format: "@a + characters(17-byte) + checksum(1-byte)"
#define CMD_PLAIN_SIZE 20

void Init_Ser_RPi(void);

#endif
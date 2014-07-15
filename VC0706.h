/*
* VC0706.h
* A library for camera shield(VC0706 chipset)
*
* Copyright (c) 2014 seeed technology inc.
* Copyright (c) 2012, Adafruit Industries.
*
* All rights reserved.
*
* This library is based on Adafruit's VC0706 Serial Camera Library. Thanks to 
* their contribution to the code, we modify it to support our Seeed's Camera 
* Shield module.
*
* Below is the introduction of Adafruit's VC0706 Serial Camera module, we add it to here
* to express our thanks to them.
*
* ****************************************************************************
* This is a library for the Adafruit TTL JPEG Camera (VC0706 chipset)
*
* Pick one up today in the adafruit shop!
* ------> http://www.adafruit.com/products/397
*
* These displays use Serial to communicate, 2 pins are required to interface
*
* Adafruit invests time and resources providing this open source code, 
* please support Adafruit and open-source hardware by purchasing 
* products from Adafruit!
*
* Written by Limor Fried/Ladyada for Adafruit Industries.  
* BSD license, all text above must be included in any redistribution
* ******************************************************************************
*
* Software License Agreement (BSD License)
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holders nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef _VC0706_H_
#define _VC0706_H_

#include "Arduino.h"
#include "SoftwareSerial.h"

#define VC0706_PTL_BYTE             0x00
#define VC0706_SERIAL_BYTE          0x01
#define VC0706_CMD_BYTE             0x02
#define VC0706_STATUS_BYTE          0x03

#define VC0706_GET_VERSION          0x11
#define VC0706_SET_SERIAL_NUMBER    0x21
#define VC0706_SET_PORT             0x24
#define VC0706_SYSTEM_RESET         0x26
#define VC0706_READ_DATA            0x30
#define VC0706_WRITE_DATA           0x31
#define VC0706_READ_FBUF            0x32
#define VC0706_WRITE_FBUF           0x33
#define VC0706_GET_FBUF_LEN         0x34
#define VC0706_FBUF_CTRL            0x36
#define VC0706_COMM_MOTION_CTRL     0x37
#define VC0706_COMM_MOTION_STATUS   0x38
#define VC0706_COMM_MOTION_DETECTED 0x39
#define VC0706_MIRROR_CTRL          0x3A
#define VC0706_MIRROR_STATUS        0x3B
#define VC0706_COLOR_CTRL           0x3C
#define VC0706_COLOR_STATUS         0x3D
#define VC0706_POWER_SAVE_CTRL      0x3E
#define VC0706_POWER_SAVE_STATUS    0x3F
#define VC0706_AE_CTRL              0x40
#define VC0706_AE_STATUS            0x41
#define VC0706_MOTION_CTRL          0x42
#define VC0706_MOTION_STATUS        0x43
#define VC0706_TVOUT_CTRL           0x44
#define VC0706_OSD_ADD_CHAR         0x45
#define VC0706_DOWNSIZE_CTRL        0x54
#define VC0706_DOWNSIZE_STATUS      0x55
#define VC0706_GET_FLASH_SIZE       0x60
#define VC0706_ERASE_FLASH_SECTOR   0x61
#define VC0706_ERASE_FLASH_ALL      0x62
#define VC0706_READ_LOGO            0x70
#define VC0706_SET_BIZTMAP          0x71
#define VC0706_BATCH_WRITE          0x80

#define VC0706_STOPCURRENTFRAME     0x0
#define VC0706_STOPNEXTFRAME        0x1
#define VC0706_RESUMEFRAME          0x3
#define VC0706_STEPFRAME            0x2

#define VC0706_640x480              0x00
#define VC0706_320x240              0x11
#define VC0706_160x120              0x22

#define VC0706_MOTIONCONTROL        0x0
#define VC0706_UARTMOTION           0x01
#define VC0706_ACTIVATEMOTION       0x01

#define VC0706_SET_ZOOM             0x52
#define VC0706_GET_ZOOM             0x53

#define CAMERABUFFSIZ               100
#define CAMERADELAY                 10

#define VC0706_SERIAL_NUMBER        0x00

class VC0706 {
public:
    VC0706(SoftwareSerial *ser);
    boolean begin(uint16_t baud = 38400);
    boolean reset(void);
    boolean TVon(void);
    boolean TVoff(void);
    boolean takePicture(void);
    uint8_t *readPicture(uint8_t n);
    boolean resumeVideo(void);
    uint32_t frameLength(void);
    char *getVersion(void);
    uint8_t available();
    uint8_t getDownsize(void);
    boolean setDownsize(uint8_t);
    uint8_t getImageSize();
    boolean setImageSize(uint8_t);
    boolean getMotionDetect();
    uint8_t getMotionStatus(uint8_t);
    boolean motionDetected();
    boolean setMotionDetect(boolean f);
    boolean setMotionStatus(uint8_t x, uint8_t d1, uint8_t d2);
    boolean cameraFrameBuffCtrl(uint8_t command);
    uint8_t getCompression();
    boolean setCompression(uint8_t c);
      
    boolean getPTZ(uint16_t &w, uint16_t &h, uint16_t &wz, uint16_t &hz, uint16_t &pan, uint16_t &tilt);
    boolean setPTZ(uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);

    void OSD(uint8_t x, uint8_t y, char *s);
  
private:
    uint8_t _rx, _tx;
    uint16_t baud;
    uint8_t cameraSerial;
    uint8_t camerabuff[CAMERABUFFSIZ+1];
    uint8_t bufferLen;
    uint16_t frameptr;
    
    SoftwareSerial *camera;

    boolean runCommand(uint8_t cmd, uint8_t args[], uint8_t argn, uint8_t resp, boolean flushflag = true); 
    void sendCommand(uint8_t cmd, uint8_t args[], uint8_t argn); 
    uint8_t readResponse(uint8_t numbytes, uint8_t timeout);
    boolean verifyResponse(uint8_t command);
    void printBuff(void);
};

#endif
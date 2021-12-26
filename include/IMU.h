#ifndef IMU_H_BLOCK
#define IMU_H_BLOCK

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"


void dmpDataReady();// {mpuInterrupt = true;}
class IMU{
private:
    MPU6050 mpu;

    #define INTERRUPT_PIN 22  // use pin 2 on Arduino Uno & most boards
    #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
    bool blinkState = false;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // packet structure for InvenSense teapot demo
    uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================
public:
    
    void setup();
    bool update(float*);
    void setOffsets(const int[6]);
    static float toDeg(float);
};
#endif
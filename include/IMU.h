#ifndef IMU_H_BLOCK
#define IMU_H_BLOCK

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

#define INTERRUPT_PIN 22 // use pin 2 on Arduino Uno & most boards

// Wrapper class for MPU6050 module
class IMU {
public:
    bool setup(float *);
    bool update();
    void setOffsets(const int[6]);

private:
    MPU6050 mpu;

    bool blinkState = false;

    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q; // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]            gravity vector
    // float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float *pOut;
};
#endif
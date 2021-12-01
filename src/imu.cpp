//#define DEBUG
#include "debug.h"

#include <IMU.h>

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
// volatile bool mpuInterrupt;// = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {mpuInterrupt = true;}

void IMU::setup() {
    DPRINTLN("Test debug print in imucpp");
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    DPRINTLN(F("Initializing I2C devices..."));
    mpu.initialize();
    //pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    DPRINTLN(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    DPRINTLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        DPRINTLN(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        DPRINT(F("Enabling interrupt detection (Arduino external interrupt "));
        DPRINT(digitalPinToInterrupt(INTERRUPT_PIN));
        DPRINTLN(F(")..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        DPRINTLN(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DPRINT(F("DMP Initialization failed (code "));
        DPRINT(devStatus);
        DPRINTLN(F(")"));
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

bool IMU::update(float* out) { //returns true if valid packet is found
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        DPRINT("ypr\t");
        DPRINT(*ypr * 180/M_PI);
        DPRINT("\t");
        DPRINT(*(ypr+1) * 180/M_PI);
        DPRINT("\t");
        DPRINTLN(*(ypr+2) * 180/M_PI);
        out[0] = ypr[0] ;
        out[1] = ypr[1] ;
        out[2] = ypr[2] ;
        return true;
    }
    return false;
}

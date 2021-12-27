//#define DEBUG
#include "debug.h"

#include <IMU.h>
const int imuOffsets[6] = {-3384, -2533, 1151, 60, -6, 22};

void IMU::setOffsets(const int TheOffsets[6])
  { mpu.setXAccelOffset(TheOffsets [0]);
    mpu.setYAccelOffset(TheOffsets [1]);
    mpu.setZAccelOffset(TheOffsets [2]);
    mpu.setXGyroOffset (TheOffsets [3]);
    mpu.setYGyroOffset (TheOffsets [4]);
    mpu.setZGyroOffset (TheOffsets [5]);
  } // SetOffsets


void IMU::setup(double* out) {
    pOut = out; //address to write pitch to
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

    setOffsets(imuOffsets);

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
        attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);
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

bool IMU::update() { //returns true if valid packet is found
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        *pOut = ypr[1]*57.2957795131;
        return true;
    }
    return false;
}

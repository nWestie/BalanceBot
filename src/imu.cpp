// #define DEBUG
#include "debug.h"
// -5372.00000,  -2575.00000,  2945.00000,  58.00000,  4.00000,  -15.00000
#include <IMU.h>
// const int imuOffsets[] = {-3384, -2533, 1151, 60, -6, 22};
const int imuOffsets[] = {-3363, -2506, 1162, 56, 3, -30}; // new values

void IMU::setOffsets(const int TheOffsets[6]) {
    mpu.setXAccelOffset(TheOffsets[0]);
    mpu.setYAccelOffset(TheOffsets[1]);
    mpu.setZAccelOffset(TheOffsets[2]);
    mpu.setXGyroOffset(TheOffsets[3]);
    mpu.setYGyroOffset(TheOffsets[4]);
    mpu.setZGyroOffset(TheOffsets[5]);
} // SetOffsets

// Set up the IMU. receives the float to store pitch output into, returns true if successful
bool IMU::setup(float *pitchOutput) {

    pOut = pitchOutput; // address to write pitch to
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    if (!mpu.testConnection())
        return false;

    // load and configure the DMP
    DPRINTLN(F("Initializing DMP..."));
    uint16_t devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // setOffsets(imuOffsets);
        // Calibration Time: generate offsets and calibrate the MPU6050
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        DPRINTLN(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // dmpReady = true;

        // get expected DMP packet size for later comparison
        // packetSize = mpu.dmpGetFIFOPacketSize();
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        DPRINT(F("DMP Initialization failed (code "));
        DPRINT(devStatus);
        DPRINTLN(F(")"));
        return false;
    }
}

// returns true if valid packet is found
bool IMU::update() {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // mpu.dmpGetGravity(&gravity, &q);

        // pitch: (nose up/down, about Y axis)
        *pOut = atan2(2 * q.x * q.z - 2 * q.w * q.y, q.z * q.z - q.y * q.y - q.x * q.x + q.w * q.w) * 57.2957795131f;
        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // *pOut = ypr[1] * 57.2957795131f;
        return true;
    }
    return false;
}

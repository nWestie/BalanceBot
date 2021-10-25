#include "Arduino.h"
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//MPU6050 defines
#define INTERRUPT_PIN 22  // use pin 2 on Arduino Uno & most boards
#define I2CDEV_IMPLEMENTATION       I2CDEV_TEENSY_3X_WIRE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
    mpuInterrupt = true;
}
class IMU {
    MPU6050 mpu;
    private: 
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
    public:
        int init(){
            mpu.initialize();
            pinMode(INTERRUPT_PIN, INPUT);
            
            Wire.begin();
            Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
            
            Serial.println(F("Testing device connections..."));
            Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

            Serial.println(F("Initializing DMP..."));
            devStatus = mpu.dmpInitialize();

            //mpu.setXGyroOffset(220);
            //mpu.setYGyroOffset(76);
            //mpu.setZGyroOffset(-85);
            //mpu.setZAccelOffset(1788);
            if(devStatus != 0){
                Serial.print(F("DMP Initialization failed (code "));
                Serial.print(devStatus);
                Serial.println(F(")"));
                return devStatus;
            }
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // enable Arduino interrupt detection
            Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
            Serial.print(INTERRUPT_PIN);
            Serial.println(F(")..."));
            attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
            if(!dmpReady) return 2;
            return 0;
        }
        void update(){
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("ypr\t");
                Serial.print(ypr[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(ypr[2] * 180/M_PI);
            }
        }
};


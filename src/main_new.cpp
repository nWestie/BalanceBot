#define DEBUG

#include <Arduino.h>
#include <Encoder.h>

#include "debug.h"
#include <myPID.h>
#include "IMU.h"

#define LEDPIN 21

volatile bool imu_interrupt = false;
void imuInterruptCallback() { imu_interrupt = true; }

class Timer
{
    const uint16_t intervalMillis;
    uint32_t nextTriggerTime;

public:
    Timer(uint16_t interval) : intervalMillis(interval), nextTriggerTime(millis()) {}

    // if timer is expired, returns true and resets the timer.
    bool checkTimer()
    {
        uint32_t t = millis();
        if (t < nextTriggerTime)
        {
            return false;
        }
        nextTriggerTime = t + intervalMillis;
        return true;
    }
};

class Battery
{
public:
    Battery(uint8_t pin_name) : pin(pin_name)
    {
        float reading = analogRead(pin) * battMult;
        for (int i = 0; i < 8; i++)
            smooth[i] = reading;
        voltageAvg = reading * 8;
    };
    // should be called at a regular interval
    void updateVoltage()
    {
        btInd = btTailInd;
        btTailInd = (btTailInd + 1) & 0x07; // loops til 7 then 'overflows'

        smooth[btInd] = analogRead(pin) * battMult;
        voltageAvg += smooth[btInd];
        voltageAvg -= smooth[btTailInd];
    };
    double getVoltage() { return voltageAvg; }
    // true if voltage is below threshold of 11.2v
    boolean lowVoltage() { return voltageAvg < 11.2; }

private:
    const float battMult = (3.3 * 12.9) / (3 * 1024) / 8; // accounts for voltage devider, analog input range, and averaging process
    const uint8_t pin;

    float smooth[8];
    uint8_t btInd = 0;     // index of newest value
    uint8_t btTailInd = 1; // index of oldest value

    float voltageAvg;
};

class Motor
{
private:
    const uint8_t dirPin;
    const uint8_t pwmPin;

public:
    const Encoder enc;
    const PID pid;
    Motor(uint8_t pwm, uint8_t dir, uint8_t encoder1, uint8_t encoder2) : dirPin(dir), pwmPin(pwm), enc(encoder1, encoder2) {}

    void drive(byte pwm, byte dir)
    {
        digitalWriteFast(dirPin, dir);
        analogWrite(pwmPin, pwm);
    }
};

class Bot
{
public:
    Bot()
    {
        imu.setup(&measuredPitch);
    }

    void runControl()
    {
        if (imu_interrupt)
        {
            imu_interrupt = false;
            imu.update();
        }
        else if (battTimer.checkTimer())
        {
            batt.updateVoltage();
            enabled = enabled && !batt.lowVoltage(); // disable if low voltage
        }
    }
    void sendData()
    {
    }

private:
    Timer battTimer = Timer(100);
    Battery batt = Battery(20);
    Motor lMotor = Motor(17, 15, 7, 8); // pwm, dir, enc1, enc2
    Motor rMotor = Motor(16, 14, 5, 6);
    IMU imu;

    float measuredPitch;
    bool enabled = false;

    // stops both motors
    void fullStop()
    {
        lMotor.drive(0, 1);
        rMotor.drive(0, 1);
    }
};
Bot *bot;
void setup()
{
    Serial.begin(38400);
    for (byte i = 5; i < 9; i++)
        pinMode(i, INPUT);
    for (byte i = 14; i < 18; i++)
        pinMode(i, OUTPUT);

    pinMode(LEDPIN, OUTPUT);

    attachInterrupt(22, imuInterruptCallback, RISING); // interrupt pin for the mpu

    bot = new Bot();
}
void loop()
{
    /// loop over all the timers and do the shit when da shit needs done
    bot->runControl();
}
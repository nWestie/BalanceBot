#define DEBUG

#include <Arduino.h>
#include <Encoder.h>

#include "debug.h"
#include "IMU.h"

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
        btTailInd = ++btTailInd & 0x07; // loops til 7 then 'overflows'

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
    const uint8_t dirPin, pwmPin;

public:
    Encoder enc;
    Motor(uint8_t pwm, uint8_t dir, uint8_t encoder1, uint8_t encoder2) : enc(encoder1, encoder2), dirPin(dir), pwmPin(pwm) {}

    void drive(byte pwm, byte dir)
    {
        digitalWriteFast(dirPin, dir);
        analogWrite(pwmPin, pwm);
    }
};

class Bot
{
    float measuredPitch;

    Battery batt = Battery(20);
    Motor lMotor = Motor(17, 15, 7, 8); // pwm, dir, enc1, enc2
    Motor rMotor = Motor(16, 14, 5, 6);
    IMU imu;

    void fullStop()
    {
        lMotor.drive(0, 1);
        rMotor.drive(0, 1);
    }

public:
    Bot()
    {
        imu.setup(&measuredPitch);
    }

    void RunBot()
    {
    }
    // stops both motors
};
Bot *bot;
void setup()
{
    bot = new Bot();
}
void loop()
{
    bot->RunBot();
}
#define DEBUG

#include <Arduino.h>
#include <Encoder.h>

#include "IMU.h"
#include "bt.h"
#include "debug.h"
#include "myPID.h"

#define LEDPIN 21

class Timer {
    const uint16_t intervalMillis;
    uint32_t nextTriggerTime;

public:
    // Interval in milliseconds
    Timer(uint16_t interval) : intervalMillis(interval), nextTriggerTime(millis() + interval) {}

    // if timer is expired, returns true and resets the timer.
    bool hasTimedOut() {
        uint32_t t = millis();
        if (t < nextTriggerTime) {
            return false;
        }
        nextTriggerTime = t + intervalMillis;
        return true;
    }
};

// accounts for voltage divider, analog input range, and averaging process
constexpr float battMult = (3.3 * 12.9) / (3 * 1024) / 8;
class Battery {
public:
    Battery(uint8_t pin_name) : pin(pin_name) {
        float reading = analogRead(pin) * battMult;
        for (int i = 0; i < 8; i++)
            smooth[i] = reading;
        voltageAvg = reading * 8;
    };

    // should be called at a regular interval
    void updateVoltage() {
        btInd = btTailInd;
        btTailInd = (btTailInd + 1) & 0x07; // loops til 7 then starts back at 0

        smooth[btInd] = analogRead(pin) * battMult;
        voltageAvg += smooth[btInd];
        voltageAvg -= smooth[btTailInd];
    };

    double getVoltage() { return voltageAvg; }

    // true if voltage is below threshold of 11.2v
    boolean lowVoltage() { return voltageAvg < 11.2; }

private:
    const uint8_t pin;

    float smooth[8];
    uint8_t btInd = 0;     // index of newest value
    uint8_t btTailInd = 1; // index of oldest value

    float voltageAvg;
};

class Motor {
private:
    const uint8_t dir;
    const uint8_t pwm;

public:
    const Encoder enc;

    Motor(uint8_t pwmPin, uint8_t dirPin, uint8_t encoder1, uint8_t encoder2) : dir(dirPin), pwm(pwmPin), enc(encoder1, encoder2) {}

    void drive(byte pwm, byte dir) {
        digitalWriteFast(dir, dir);
        analogWrite(pwm, pwm);
    }
};

bool enabled = false;

Timer battTimer(100);
Battery batt(20);

PID::KPID motorPID = {40.0, 1, .05};
// Motor lMotor = Motor(17, 15, 7, 8); // pwm, dir, enc1, enc2
// Motor rMotor = Motor(16, 14, 5, 6);

// stops both motors
// void fullStop()
// {
// lMotor.drive(0, 1);
// rMotor.drive(0, 1);
// }

// BT things
void updatePID(PID::KPID &) {}
void savePID(PID::KPID &) {}
BTHandler bt = BTHandler(updatePID, savePID, motorPID);

// IMU things
IMU imu;
volatile bool imu_interrupt = false;
void imuInterruptCallback() { imu_interrupt = true; }
float measuredPitch;

Timer updateControl(10);
Timer updateStats(50);

void setup() {
    Serial.begin(38400);
    // set inputs
    for (byte i = 5; i < 9; i++)
        pinMode(i, INPUT);
    // set outputs
    for (byte i = 14; i < 18; i++)
        pinMode(i, OUTPUT);
    pinMode(LEDPIN, OUTPUT);

    attachInterrupt(22, imuInterruptCallback, RISING); // interrupt pin for the mpu

    digitalWriteFast(LEDPIN, HIGH);
    imu.setup(&measuredPitch);
    digitalWriteFast(LEDPIN, LOW);
}

Timer flashTimer(500);

void loop() {
    /// loop over all the timers and do the shit when da shit needs done
    if (battTimer.hasTimedOut()) // 10 Hz
    {
        batt.updateVoltage();
        // disable if low voltage
        if (enabled && batt.lowVoltage())
            ;
    }

    if (imu_interrupt) {
        imu_interrupt = false;
        imu.update();
    }

    if (updateControl.hasTimedOut()) // runs at 100 Hz
    {
        String s = bt.recDataTest();
        if (s.length()) {
            Serial.print("Len(");
            Serial.print(s.length());
            Serial.print("): ");
            Serial.print(s);
            for (unsigned int i = 0; i < s.length(); i++) {
                Serial.print(s[i], HEX);
                Serial.print(", ");
            }
            Serial.println();
        }
    }
    if (!enabled) {
        if (flashTimer.hasTimedOut()) // 2 Hz, 500ms on/off
        {
            digitalWriteFast(LEDPIN, !digitalReadFast(LEDPIN)); // toggle LED
            bt.print(String(batt.getVoltage()) + "\n");
        }
        return;
    }
}

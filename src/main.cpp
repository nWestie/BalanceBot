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

class Battery {
public:
    Battery(uint8_t pin_name) : pin(pin_name) {}

    //  returns the averaged voltage
    float getVoltage() { return analogRead(pin) * battMult; }

    // true if voltage is below threshold of 11.0V
    boolean lowVoltage(float voltage) { return voltage < 11.0; }

private:
    const uint8_t pin;
    // accounts for voltage divider, analog input range, and averaging process
    const float battMult = (3.3 * 12.9) / (3 * 1024);
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

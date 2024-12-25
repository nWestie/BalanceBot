#define DEBUG

#include <Arduino.h>
#include <Encoder.h>

#include "IMU.h"
#include "bt.h"
#include "debug.h"
#include "myPID.h"
#include <EEPROM.h>

#define LEDPIN 21
#define EEPROMADDR 32
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

    //  returns the battery voltage
    float getVoltage() { return analogRead(pin) * battMult; }

    // true if voltage is below threshold of 11.0V
    boolean lowVoltage(float voltage) { return voltage < 11.0; }

private:
    const uint8_t pin;
    // accounts for voltage divider and analog input range
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

Battery batt(20);

PID::KPID motorPID = {0, 0, 0};
float measuredPitch, power, pitchSet;
PID pidControl(motorPID, &measuredPitch, &power, &pitchSet, REVERSE);

// Motor lMotor = Motor(17, 15, 7, 8); // pwm, dir, enc1, enc2
// Motor rMotor = Motor(16, 14, 5, 6);

// stops both motors
// void fullStop()
// {
// lMotor.drive(0, 1);
// rMotor.drive(0, 1);
// }

// BT things
void enableReceived(bool);
// Send PID to PID controller
void updatePID(PID::KPID pid) { pidControl.SetTunings(pid); }
// Save PID to EEPROM
void savePID(PID::KPID pid) { EEPROM.put(EEPROMADDR, pid); }
// Fetch PID from EEPROM
PID::KPID recallPID() {
    PID::KPID vals;
    return EEPROM.get(EEPROMADDR, vals);
}

BTHandler bt = BTHandler(updatePID, savePID, enableReceived, motorPID);

// IMU things
IMU imu;
volatile bool imu_interrupt = false;
void imuInterruptCallback() { imu_interrupt = true; }

// Timer for control loop
Timer updateControl(10);
// Timer sendVoltage(200); // sends the phone specific voltage updates

void setup() {
    Serial.begin(38400);
    // set inputs
    for (byte i = 5; i < 9; i++)
        pinMode(i, INPUT);
    // set outputs
    for (byte i = 14; i < 18; i++)
        pinMode(i, OUTPUT);
    pinMode(LEDPIN, OUTPUT);

    motorPID = recallPID();
    attachInterrupt(22, imuInterruptCallback, RISING); // interrupt pin for the mpu

    digitalWriteFast(LEDPIN, HIGH);
    imu.setup(&measuredPitch);
    digitalWriteFast(LEDPIN, LOW);
}

enum class RunState {
    IDLE,
    ENABLING,
    RUNNING,
    DISABLING
};
RunState state = RunState::IDLE;

Timer flashTimer(500);
void loop() {

    /// loop over all the timers and do the shit when da shit needs done
    if (imu_interrupt) {
        imu_interrupt = false;
        imu.update();
    }
    // if (sendVoltage.hasTimedOut()) {
    //     bt.sendBatt(batt.getVoltage());
    // }
    bt.receiveData();
    // TODO: Check low voltage
    if (updateControl.hasTimedOut()) // runs at 100 Hz
    {
        float volt = batt.getVoltage();
        bt.sendUpdate(volt, 90, measuredPitch, state == RunState::RUNNING);

        switch (state) {
        case RunState::IDLE:
            if (flashTimer.hasTimedOut()) // 2 Hz, 500ms on/off
            {
                Serial.println("Idling...");
                digitalWriteFast(LEDPIN, !digitalReadFast(LEDPIN)); // toggle LED
            }
            break;
        case RunState::ENABLING:
            digitalWriteFast(LEDPIN, LOW); // Make sure LED is off
            Serial.println("Enabling...");
            state = RunState::RUNNING;
            break;
        case RunState::RUNNING:
            if (measuredPitch < 40 || measuredPitch > 130){// halts if robot tips over
                state = RunState::DISABLING;
                bt.print("Disabling, fell over");
            }
            if (batt.lowVoltage(volt)) {
                bt.print("Disabling, low voltage");
                state = RunState::DISABLING;
            }
            break;
        case RunState::DISABLING:
            Serial.println("Disabling...");
            state = RunState::IDLE;
            break;
        }
    }
}

// Called when BT module receives enable or disable signal, transitions state accordingly
void enableReceived(bool enable) {
    if (enable and state != RunState::RUNNING)
        state = RunState::ENABLING;
    else if (state != RunState::IDLE)
        state = RunState::DISABLING;
}
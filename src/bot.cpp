#include <Arduino.h>
#include <EEPROM.h>
#include "bot.h"
#include "bt.h"

Bot::Bot(BTInterface *bt)
{
    EEPROM.get(1, _pid);
    _pidControl.SetTunings(_pid.p, _pid.i, _pid.d);
    _pidControl.SetOutputLimits(-255, 255);
    _pidControl.SetSampleTime(10); // in ms
    _batt = batt(20, waitForEnable);

    bt->sendPID(_pid);
}

void Bot::setPID(PIDvals pid)
{
    _pid = pid;
    _pidControl.SetTunings(_pid.p, _pid.i, _pid.d);
}

Bot::PIDvals Bot::getPID() { return PIDvals(); }

void Bot::savePID() { EEPROM.put(1, _pid); }

Battery::Battery(uint8_t pin, void lowBattFunc(String str))
  {
    this->pin = pin;
    this->lowBattFunc = lowBattFunc;

    btInd = 0;
    battSmooth[0] = analogRead(pin) * battMult;
    for (int i = 1; i < 8; i++)
      battSmooth[i] = battSmooth[0];
  };
double Battery::updateVoltage(bool isEnabled)

{
    battSmooth[btInd] = analogRead(pin) * battMult;
    btInd++;
    btInd &= 0x07; // loops over above index 7 ('overflows')

    float battVoltAvg = 0;
    for (int i = 0; i < 8; i++)
        battVoltAvg += battSmooth[i];
    battVoltAvg /= 8;

    if (isEnabled && battVoltAvg < 11.18) //~20%. Don't call if disabled, to prevent call stack overflow
        lowBattFunc("LOW BATT");

    return battVoltAvg;
};
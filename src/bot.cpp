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

    bt->sendPID(_pid);
}

void Bot::setPID(PIDvals pid)
{
    _pid = pid;
    _pidControl.SetTunings(_pid.p, _pid.i, _pid.d);
}

Bot::PIDvals Bot::getPID() { return PIDvals(); }

void Bot::savePID() { EEPROM.put(1, _pid); }

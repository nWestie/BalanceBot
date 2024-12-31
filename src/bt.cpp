#define DEBUG
#include "debug.h"

#include <Arduino.h>

#include "bt.h"
#include "myPID.h"

BTHandler::BTHandler(void updatePID(PID::KPID), void savePID(PID::KPID), void onEnable(bool), PID::KPID &pid) : onEnable(onEnable), PIDupdate(updatePID), PIDsave(savePID), PIDvals(pid) {
    receivedFlag = false;
    btDataString = "";
    heartbeat = false;
    lastPacketTime = millis();

    Serial2.begin(38400);
    // Serial BT should not wait for characters when reading
    Serial2.setTimeout(0);
}
void BTHandler::sendUpdate(float voltage, float setAngle, float measuredAngle, bool isEnabled) {
    Serial2.print("U");
    Serial2.print(millis());
    Serial2.print(",");
    Serial2.print(isEnabled);
    Serial2.print(",");
    Serial2.print(voltage);
    Serial2.print(",");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print(EOMchar);
}
void BTHandler::sendPID(PID::KPID &vals) {
    Serial2.print("P");
    Serial2.print(millis());
    Serial2.print(",");
    Serial2.print(vals.p, 3);
    Serial2.print(",");
    Serial2.print(vals.i, 3);
    Serial2.print(",");
    Serial2.print(vals.d, 3);
    Serial2.print(EOMchar);
};
void BTHandler::receiveData() {
    if (!Serial2.available())
        return;
    btDataString += Serial2.readString();
    int endCharIndex = btDataString.indexOf(EOMchar);
    if (btDataString.length() > 127)
        Serial.println("WARN - large read buffer");

    while (endCharIndex != -1) {
        String packet = btDataString.substring(0, endCharIndex);
        btDataString = btDataString.substring(endCharIndex + 1);

        endCharIndex = btDataString.indexOf(EOMchar);

        lastPacketTime = millis(); // keep track of last good message for heartbeat.
        switch (packet[0])         // switch on first character
        {
        case 'X': // Get joystick commands
            ctlData.turn = packet.substring(1).toInt();
            packet = packet.substring(packet.indexOf(',') + 2);
            ctlData.speed = packet.toInt();
            break;
        case 'T': // Get Trim commands
            ctlData.trim = packet.substring(1).toFloat();
            Serial.println(ctlData.trim, 2);
            break;
        case 'E':
            onEnable(true);
            break;
        case 'D':
            onEnable(false);
            break;
        case 'S': // save PID to EEprom
        case 'P': // read updated PID values
            PIDvals.p = packet.substring(1).toFloat();
            PIDvals.i = packet.substring(packet.indexOf(',') + 1).toFloat();
            PIDvals.d = packet.substring(packet.lastIndexOf(',') + 1).toFloat();
            this->PIDupdate(PIDvals);
            if (packet[0] == 'S') {
                this->PIDsave(PIDvals);
                print("PID Saved to EEPROM\n");
            }
            sendPID(PIDvals);
            break;
        case 'R': // send requested PID vals
            sendPID(PIDvals);
            break;
        }
    }
    heartbeat = (millis() - lastPacketTime) < 200;
    return;
};

CTLData BTHandler::getCTL() { return ctlData; }
void BTHandler::sendBatt(float voltage) { Serial2.print("V" + String(voltage) + "/"); }
void BTHandler::print(String str) {
    Serial2.print("M" + str + "/");
};
#define DEBUG
#include "debug.h"

#include <Arduino.h>

#include "bt.h"
#include "myPID.h"

// 'random' codes for disable an enable, to reduce chance of false triggers
const String ENABLECODE = "213";
const String DISABLECODE = "226";

KivyBT::KivyBT(void updatePID(), void savePID(), PID::KPID *pid)
{
    this->PIDupdate = updatePID;
    this->PIDsave = savePID;
    this->PIDvals = pid;

    receivedFlag = false;
    btDataString = "";
    connected = false;
    Serial2.begin(38400);
    Serial2.setTimeout(0);
}
void KivyBT::sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled)
{
    Serial2.print("U");
    Serial2.print(millis());
    Serial2.print(",");
    Serial2.print(voltage);
    Serial2.print(",");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print(",");
    Serial2.print(isEnabled);
    Serial2.print(EOMchar);
}
void KivyBT::sendPID()
{
    Serial2.print("P");
    Serial2.print(millis());
    Serial2.print(",");
    Serial2.print(PIDvals->p, 3);
    Serial2.print(",");
    Serial2.print(PIDvals->i, 3);
    Serial2.print(",");
    Serial2.print(PIDvals->d, 3);
    Serial2.print(EOMchar);
};
bool KivyBT::receiveData(BTData *recBTData)
{
    bool dataUpdated = false; // will only return true if new data is added to recBTData

    int bytes = Serial2.available();
    char newData[bytes];
    Serial2.readBytes(newData, bytes);
    btDataString += newData;
    if (btDataString.length() > 255)
        btDataString = "";
    int endCharIndex = btDataString.indexOf(EOMchar);
    while (endCharIndex != -1)
    {
        String packet = btDataString.substring(0, endCharIndex);
        btDataString = btDataString.substring(endCharIndex + 1);

        endCharIndex = btDataString.indexOf(EOMchar);

        // remove any non-message characters before message
        if (!isAlpha(packet[0]))
        {
            uint16_t dataStart = 1;
            while (dataStart < packet.length())
            {
                if (isAlpha(packet[dataStart]))
                    break;
                dataStart++;
            }
            // if there are no good characters
            if (dataStart == packet.length() - 1)
            {
                DPRINTLN("Bad Packet");
                continue;
            }
            packet = packet.substring(dataStart);
        }
        // now considering packet to be only good, expected chars
        dataUpdated = true;

        lastPacketTime = millis(); // keep track of last good message for heartbeat.

        int tmpEnable;
        switch (packet[0]) // switch on first character
        {
        case 'U': // save new commands to
            recBTData->speed = packet.substring(1).toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->turn = packet.toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->trim = packet.toFloat();
            packet = packet.substring(packet.indexOf(',') + 1, packet.indexOf(',') + 4);
            // If packet is neither enable nor disable
            if (!(packet == ENABLECODE or packet == DISABLECODE))
            {
                print("Bad E Code\n");
                break;
            }
            tmpEnable = (packet == ENABLECODE) ? 1 : 0;
            if (tmpEnable != recBTData->enable)
            {
                recBTData->enable = tmpEnable;
                ackEnable();
            }
            break;
        case 'S': // save PID to EEprom
            this->PIDsave();
            break;
        case 'P': // read updated PID values
            PID::KPID vals;
            vals.p = packet.substring(1).toFloat();
            vals.i = packet.substring(packet.indexOf(',') + 1).toFloat();
            vals.d = packet.substring(packet.lastIndexOf(',') + 1).toFloat(); // TODO this might not work(lastInd)
            this->PIDupdate();
            sendPID();
            break;
        case 'R': // send requested PID vals
            sendPID();
            break;
        }
    }
    connected = (millis() - lastPacketTime) < 200;
    return dataUpdated;
};

String KivyBT::recDataTest()
{
    if (!Serial2.available())
        return "";
    btDataString += Serial2.readString();
    Serial.println(btDataString);
    int endCharIndex = btDataString.indexOf(EOMchar);
    if (endCharIndex != -1)
    {
        String packet = btDataString.substring(0, endCharIndex);
        btDataString = btDataString.substring(endCharIndex + 1);
        return packet;
    }
    return "";
};
void KivyBT::print(String str)
{
    Serial2.printf("M");
    Serial2.print(str);
    Serial2.print("/");
};
void KivyBT::ackEnable()
{
    Serial2.print("A/");
}
// --------------Phone--------------
// void PhoneBT::update(double voltage, double setAngle, double measuredAngle)
// {
//     Serial2.print("*V");
//     Serial2.print(voltage);
//     Serial2.print("**C");
//     Serial2.print(voltage);
//     Serial2.print("**G");
//     Serial2.print(setAngle);
//     Serial2.print(",");
//     Serial2.print(measuredAngle);
//     Serial2.print("*");
// }
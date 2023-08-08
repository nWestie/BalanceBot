// #define DEBUG

#include "Arduino.h"
#include "debug.h"
#include "bot.h"
#include "bt.h"

// 'random' codes for disable an enable, to reduce chance of false triggers
const String ENABLECODE = "213";
const String DISABLECODE = "226";

KivyBT::KivyBT(void updatePID(), void savePID())
{
    receivedFlag = false;
    btDataString = "";
    connected = false;
    this->PIDupdate = updatePID;
    this->PIDsave = savePID;
    Serial2.begin(38400);
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
void KivyBT::sendPID(Bot::PIDvals pid)
{
    Serial2.print("P");
    Serial2.print(millis());
    Serial2.print(",");
    Serial2.print(pid.p, 3);
    Serial2.print(",");
    Serial2.print(pid.i, 3);
    Serial2.print(",");
    Serial2.print(pid.d, 3);
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
        if (!isAlpha(packet[0]))
        {
            uint16_t dataStart = 1;
            while (dataStart < packet.length())
            {
                if (isAlpha(packet[dataStart]))
                    break;
                dataStart++;
            }
            if (dataStart == packet.length() - 1)
            {
                DPRINTLN("Bad Packet");
                continue;
            }
            packet = packet.substring(dataStart);
        }
        dataUpdated = true;
        lastPacketTime = millis();
        int tmpE;
        switch (packet[0])
        {
        case 'U':
            recBTData->speed = packet.substring(1).toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->turn = packet.toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->trim = packet.toFloat();
            packet = packet.substring(packet.indexOf(',') + 1, packet.indexOf(',') + 4);
            if (!(packet == ENABLECODE or packet == DISABLECODE))
            {
                print("Bad E Code\n");
                break;
            }
            tmpE = (packet == ENABLECODE) ? 1 : 0;
            if (tmpE != recBTData->enable)
            {
                recBTData->enable = tmpE;
                ackEnable();
            }
            break;
        case 'S':
            this->PIDsave();
            break;
        case 'P':
            Bot::PIDvals vals;
            vals.p = packet.substring(1).toFloat();
            vals.i = packet.substring(packet.indexOf(',') + 1).toFloat();
            vals.d = packet.substring(packet.lastIndexOf(',') + 1).toFloat(); // TODO this might not work(lastInd)
            this->PIDupdate();
            // sendPID(); TODO fix
            break;
        case 'R':
            // sendPID(); TODO fix
            break;
        }
    }
    connected = (millis() - lastPacketTime) < 200;
    return dataUpdated;
};
void KivyBT::print(String str)
{
    Serial2.print("M");
    Serial2.print(str);
    Serial2.print("/");
};
void KivyBT::ackEnable()
{
    Serial2.print("A/");
}
// --------------Phone--------------
void PhoneBT::update(double voltage, double setAngle, double measuredAngle)
{
    Serial2.print("*V");
    Serial2.print(voltage);
    Serial2.print("**C");
    Serial2.print(voltage);
    Serial2.print("**G");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print("*");
}
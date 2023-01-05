#define DEBUG

#include "Arduino.h"
#include "bt.h"
#include "debug.h"

KivyBT::KivyBT(double *kPID, void updatePID(), void savePID())
{
    receivedFlag = false;
    btDataString = "";

    this->kPID = kPID;
    this->PIDupdate = updatePID;
    this->PIDsave = savePID;
    Serial2.begin(38400);
}
void KivyBT::sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled)
{
    // DPRINTLN("begin Send");
    Serial2.print("U");
    Serial2.print(voltage);
    Serial2.print(",");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print(",");
    Serial2.print(isEnabled);
    Serial2.print(EOMchar);
    // DPRINTLN("exitSend");
}
void KivyBT::sendPID()
{
    Serial2.print("P");
    Serial2.print(kPID[0]);
    Serial2.print(",");
    Serial2.print(kPID[1]);
    Serial2.print(",");
    Serial2.print(kPID[2]);
    Serial2.print(EOMchar);
};
bool KivyBT::receiveData(BTData *recBTData) // TODO: will need SIGNIFICANT testing
{
    bool dataUpdated = false; // will only return true if new data is added to recBTData
    bool PIDUpdated = false;  // only call PIDUpdate once when multiple PID vals change

    int bytes = Serial2.available();
    char newData[bytes];
    Serial2.readBytes(newData, bytes);
    btDataString += newData;

    int endCharIndex = btDataString.indexOf(EOMchar);
    while (endCharIndex != -1)
    {
        String packet = btDataString.substring(0, endCharIndex);
        btDataString = btDataString.substring(endCharIndex + 1);
        endCharIndex = btDataString.indexOf(EOMchar);
        
        if (!isAlpha(packet[0]))
        {
            uint16_t dataStart = 1;
            while(dataStart < packet.length())
            {
                if(isAlpha(packet[dataStart]))
                    break;
                dataStart ++;
            }
            if(dataStart==packet.length()-1)
                continue;
            packet = packet.substring(dataStart);
        }
        dataUpdated = true;
        switch (packet[0])
        {
        case 'U':
            recBTData->speed = packet.substring(1).toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->turn = packet.toInt();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->trim = packet.toFloat();
            packet = packet.substring(packet.indexOf(',') + 1);
            recBTData->enable = packet.toInt();
            DPRINTLN(recBTData->speed);
            break;
        case 'S':
            this->PIDsave();
            break;
        case 'P':
            kPID[0] = packet.substring(1).toFloat();
            packet = packet.substring(packet.indexOf(',') + 1);
            kPID[1] = packet.substring(1).toFloat();
            packet = packet.substring(packet.indexOf(',') + 1);
            kPID[2] = packet.substring(1).toFloat();
            PIDUpdated = true;
            break;
        }
    }
    if (PIDUpdated)
        this->PIDupdate();
    return dataUpdated;
};
void KivyBT::print(String str)
{
    Serial2.print("M");
    Serial2.print(str);
    Serial2.print("/");
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
#include "Arduino.h"
#include "bt.h"
KivyBT::KivyBT(double *kPID, void updatePID(), void savePID())
{
    receivedFlag = false;
    btDataString = "";

    this->kPID = kPID;
    this->PIDupdate = updatePID;
    this->PIDsave = savePID;
    Serial2.begin(38400);
}
void KivyBT::update(double voltage, double setAngle, double measuredAngle)
{
    Serial2.print("U");
    Serial2.print(voltage);
    Serial2.print(",");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print(EOMchar);
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
    btDataString.concat(newData);

    uint8_t endChar = btDataString.indexOf(EOMchar);
    while (endChar != -1)
    {
        String packet = btDataString.substring(0, endChar);
        btDataString = btDataString.substring(endChar + 1);
        if (!isAlpha(packet[0]))
        {
            endChar = btDataString.indexOf(EOMchar);
            continue;
        }
        dataUpdated = true;
        switch (packet[0])
        {
        case 'U':
            recBTData->speed = packet.substring(1).toInt();
            recBTData->turn = packet.substring(packet.indexOf(',') + 1).toInt();
            break;
        case 'E':
            recBTData->enable = !recBTData->enable;
            break;
        case 'S':
            this->PIDsave();
            break;
        case 'T':
            recBTData->trim = packet.substring(1).toInt();
            break;
        case 'P':
            kPID[0] = packet.substring(1).toInt();
            PIDUpdated = true;
            break;
        case 'I':
            kPID[1] = packet.substring(1).toInt();
            PIDUpdated = true;
            break;
        case 'D':
            kPID[2] = packet.substring(1).toInt();
            PIDUpdated = true;
            break;
        }
    }
    if (PIDUpdated)
        this->PIDupdate();
    return dataUpdated;
};
void KivyBT::print(char *str)
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
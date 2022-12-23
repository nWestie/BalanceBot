#include <array>
#include <string>
#include "bt.h"
#include "Arduino.h"
KivyBT::KivyBT(double *kPID)
{
    Serial2.begin(38400);
    enabled = false;
    receivedFlag = false;
    this->kPID = kPID;
}
void KivyBT::update(double voltage, double setAngle, double measuredAngle)
{
    receiveData();
    Serial2.print("U");
    Serial2.print(voltage);
    Serial2.print(",");
    Serial2.print(setAngle);
    Serial2.print(",");
    Serial2.print(measuredAngle);
    Serial2.print("/");
}
void KivyBT::sendPID()
{
    Serial2.print("P");
    Serial2.print(kPID[0]);
    Serial2.print(",");
    Serial2.print(kPID[1]);
    Serial2.print(",");
    Serial2.print(kPID[2]);
    Serial2.print("/");
};
void KivyBT::receiveData()
{
    int bytes = Serial2.available();
    while (Serial2.available())
    {
        // switch based on which slider its from
        char sw;
        sw = Serial2.read(); // read ID
        switch (sw)
        {
        case 'X':
            steer = Serial2.parseInt();
            steer -= 255;
            steer /= 8;
            break;
        case 'Y':
        {
            int mPow = Serial2.parseInt();
            mPow -= 255;
            mPow = -mPow;
            pitchInp = 90 + (mPow / 32); // conversion to setpoint
            pitchSet = pitchInp + pitchTrim;
            break;
        }
        case 'S':
            if (enable)
            {
                stopAll();
                waitForEnable();
            }
            else
                enable = true;
            break;
        case 'F':
            pitchTrim += .1;
            pitchSet = pitchInp + pitchTrim;
            break;
        case 'B':
            pitchTrim -= .1;
            pitchSet = pitchInp + pitchTrim;
            break;
        case 'M':
        { // terminal commands
            while (!Serial2.available())
                ;
            char sw = Serial2.read();
            if (isLowerCase(sw))
            {
                sw -= 32;
            }
            double newVal = Serial2.parseFloat();
            switch (sw)
            {
            case 'P':
                kPID[0] = newVal;
                break;
            case 'I':
                kPID[1] = newVal;
                break;
            case 'D':
                kPID[2] = newVal;
                break;
            }
        }
        case '/':
            break; // End character
        }
    }
}
bool KivyBT::botEnabled()
{
    return enabled;
}
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
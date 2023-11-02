#ifndef BT_H_BLOCK
#define BT_H_BLOCK

#include <Arduino.h>
#include <array>
#include "myPID.h"

struct BTData // speed, turn, trim, enable
{
    int8_t speed; // range -128 to 127
    int8_t turn;  // range -128 to 127
    float trim;   // set point trim in deg
    bool enable;  // true if robot should be enabled
};
class BTInterface
{
public:
    // sends PID weights to controller
    virtual void sendPID() = 0;
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    virtual void sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled) = 0;
    // prints a string to controller console
    virtual void print(String str) = 0;
    // gets any new data from the controller, updating the provided struct as nessisary.
    // will call provided PIDUpdate and PIDsave as needed.
    virtual bool receiveData(BTData *recBTData) = 0;
    // virtual bool isConnected() = 0;
    virtual String recDataTest() = 0;
};
class KivyBT : public BTInterface
{
public: // TODO not sure if I need any of this? if I say it's overriding BTInter
    KivyBT(void updatePID(), void savePID(), PID::KPID *pid);
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    void sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled);
    // sends PID weights to controller
    void sendPID();
    // prints a string to controller console
    void print(String str);
    /// gets any new data from the controller, updating the provided struct as nessisary.
    /// will call provided PIDUpdate and PIDsave as needed.
    bool receiveData(BTData *recBTData);
    virtual String recDataTest();

private:
    PID::KPID *PIDvals;
    void (*PIDupdate)();
    void (*PIDsave)();
    const char EOMchar = '/'; // signifies end of all sent/recieved messages
    String btDataString;
    bool receivedFlag;
    uint32_t lastPacketTime;
    void ackEnable();
    float trim;
    bool connected;
};
// class PhoneBT : public BTInterface
// {
//     PhoneBT();
//     bool receiveData(BTData *recBTData);
//     void sendPID();
//     void sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled);
//     void print(String str);
// };

#endif
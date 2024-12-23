#ifndef BT_H_BLOCK
#define BT_H_BLOCK

#include "myPID.h"
#include <Arduino.h>
#include <array>

struct CTLData // speed, turn, trim, enable
{
    int8_t speed; // range -128 to 127
    int8_t turn;  // range -128 to 127
    float trim;   // set point trim in deg
};

class BTHandler {
public:
    BTHandler(void updatePID(PID::KPID &), void savePID(PID::KPID &), PID::KPID &pid);
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    void sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled);
    // sends PID weights to controller
    void sendPID(PID::KPID &);
    // prints a string to controller console
    void print(String str);
    /// gets any new data from the controller, updating the provided struct as necessary.
    /// will call provided PIDUpdate and PIDsave as needed.
    void receiveData();
    CTLData getCTL();
    String recDataTest();

private:
    void (*PIDupdate)(PID::KPID &);
    void (*PIDsave)(PID::KPID &);
    PID::KPID &PIDvals;
    const char EOMchar = '/'; // signifies end of all sent/received messages
    String btDataString;
    bool receivedFlag;
    uint32_t lastPacketTime;
    bool connected;
    CTLData ctlData{0, 0, 0};
};

#endif
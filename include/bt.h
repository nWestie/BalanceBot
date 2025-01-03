#ifndef BT_H_BLOCK
#define BT_H_BLOCK

#include "myPID.h"
#include <Arduino.h>
#include <array>

struct CTLData // speed, turn, trim, enable
{
    float trim;   // set point trim in deg
    int8_t speed; // range -128 to 127
    int8_t turn;  // range -128 to 127
};

class BTHandler {
public:
    BTHandler(void updatePID(PID::KPID), void savePID(PID::KPID), void onEnable(bool), PID::KPID &pid);
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    void sendUpdate(float voltage, float setAngle, float measuredAngle, bool isEnabled);
    // sends PID weights to controller
    void sendPID(PID::KPID &);
    // prints a string to controller console
    void print(String str);
    /// gets any new data from the controller, updating the provided struct as necessary.
    /// will call provided PIDUpdate and PIDsave as needed.
    void receiveData();
    CTLData getCTL();
    void sendBatt(float voltage);

private:
    void (*onEnable)(bool);       // to be called when enable is received
    void (*PIDupdate)(PID::KPID); // called to update bot PID values
    void (*PIDsave)(PID::KPID);   // Called to save bot PID values
    PID::KPID &PIDvals;           // Referece to the PID values used by the bot
    const char EOMchar = '/';     // signifies end of all sent/received messages
    String btDataString;
    bool receivedFlag;
    uint32_t lastPacketTime;
    bool heartbeat;
    CTLData ctlData{0, 0, 0};
};

#endif
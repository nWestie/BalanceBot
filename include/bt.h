#ifndef BT_H_BLOCK
#define BT_H_BLOCK

#include <Arduino.h>
#include <array>
struct BTData // speed, turn, trim, enable
{
    int8_t speed; // range -128 to 127
    int8_t turn;  // range -128 to 127
    float trim;   // setpoint trim in deg
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
    double *kPID; // updated with new PID vals
    float trim;
    bool connected;
};
class KivyBT : public BTInterface
{
public:
    KivyBT(double *kPID, void updatePID(), void savePID());
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    void sendUpdate(double voltage, double setAngle, double measuredAngle, double isEnabled);
    // sends PID weights to controller
    void sendPID();
    // prints a string to controller console
    void print(String str);
    /// gets any new data from the controller, updating the provided struct as nessisary.
    /// will call provided PIDUpdate and PIDsave as needed.
    bool receiveData(BTData *recBTData);

private:
    const char EOMchar = '/'; // signifies end of all sent/recieved messages
    String btDataString;
    bool receivedFlag;
    uint32_t lastPacketTime;
    void ackEnable();
    void (*PIDupdate)();
    void (*PIDsave)();
};
class PhoneBT : public BTInterface
{
    PhoneBT(double kPID[3]);
    void sendPID(double kPID[3]);
    void update(double voltage, double setAngle, double measuredAngle);
    void print(char *str);
};

#endif
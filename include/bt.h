#ifndef BT_H_BLOCK
#define BT_H_BLOCK

#include <Arduino.h>
#include <array>
struct BTData
{
    double trim;  // setpoint trim in deg
    int16_t speed; // range -255 to 255
    int16_t turn;  // range -255 to 255
    bool enable;  // false: signal calling code to disable/enable robot
};
class BTInterface
{
public:
    // sends PID weights to controller
    virtual void sendPID() = 0;
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    virtual void update(double voltage, double setAngle, double measuredAngle) = 0;
    // prints a string to controller console
    virtual void print(String str) = 0;
    double *kPID; // updated
    double trim;
};
class KivyBT : public BTInterface
{
public:
    KivyBT(double *kPID, void updatePID(), void savePID());
    // sends latest batt voltage, setAngle, and measured angle to controller for diagnostics
    void update(double voltage, double setAngle, double measuredAngle);
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
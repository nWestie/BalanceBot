#include <Arduino.h>
#include <array>
struct BTData{
    double p;
    double i;
    double d;
    double trim; //setpoint trim in deg
    int8_t speed; // range -128 to 127
    int8_t turn;  // range -128 to 127
};
class BTInterface
{
public:
    virtual void sendPID() = 0;
    virtual void update(double voltage, double setAngle, double measuredAngle) = 0;
    virtual void print(char *str) = 0;
    double *kPID; // updated
    double trim;

protected:
    bool (*enableBot)();
};
class KivyBT : public BTInterface
{
public:
    KivyBT(double *kPID, void updatePID(), void savePID(), bool enable());
    void update(double voltage, double setAngle, double measuredAngle);
    void sendPID();
    void print(char *str);

private:
    const char EOMchar = '/'; //signifies end of all sent/recieved messages
    String btDataString;
    bool receiveData(BTData *recBTData);
    bool receivedFlag;
    void (*PIDupdate)();
    void (*PIDsave)();
    
};
class PhoneBT : public BTInterface
{
    PhoneBT(double kPID[3]);
    void sendPID(double kPID[3]);
    void update(double voltage, double setAngle, double measuredAngle);
    bool botEnabled();
    void print(char *str);
};
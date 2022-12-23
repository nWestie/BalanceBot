#include <array>
class BTInterface
{
public:
    virtual void sendPID(double kPID[3]) = 0;
    virtual void update(double voltage, double setAngle, double measuredAngle) = 0;
    virtual bool botEnabled() = 0;
    virtual void print(std::string str) = 0;
    double *kPID; // updated
    double trim;

protected:
    bool enabled;
};
class KivyBT : public BTInterface
{
public:
    KivyBT::KivyBT(double *kPID);
    void update(double voltage, double setAngle, double measuredAngle);
    void sendPID();
    bool botEnabled();
    void print(char *str);

private:
    void receiveData();
    bool receivedFlag;
};
class PhoneBT : public BTInterface
{
    PhoneBT(double kPID[3]);
    void sendPID(double kPID[3]);
    void update(double voltage, double setAngle, double measuredAngle);
    bool botEnabled();
    void print(char *str);
};
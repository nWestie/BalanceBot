#ifndef MAIN_H_BLOCK
#define MAIN_H_BLOCK

#include "bt.h"

class Bot
{
public:
  struct PIDvals
  {
    double p;
    double i;
    double d;
  };
  Bot(BTInterface *bt);
  void setPID(PIDvals pid);
  PIDvals getPID();
  void savePID();

private:
  BTInterface *_bt;
  PIDvals _pid;
  PID _pidControl;
};

class Battery
{
public:
  Battery(uint8_t pin, void lowBattFunc(String str));
  double updateVoltage(bool isEnabled)

private:
  const float battMult = (3.3 * 12.9) / (3 * 1024); // converts from read voltage to battery voltage
  uint8_t pin;
  float battSmooth[8];
  uint8_t btInd;
  void (*lowBattFunc)(String str);
};
#endif
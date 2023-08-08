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

#endif
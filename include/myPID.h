#ifndef myPID
#define myPID

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

class PID
{
public:
  // Constants used in some of the functions below
  struct KPID
  {
    double p;
    double i;
    double d;
  };

  // commonly used functions:
  // constructor.  links the PID to the Input, Output, and
  // Setpoint.  Initial tuning parameters are also set here.
  // (overload for specifying proportional mode)
  PID(KPID, double *, double *, double *, int, int);

  void Compute(); // * performs the PID calculation.  it should be
                  //   called every time loop() cycles. ON/OFF and
                  //   calculation frequency can be set using SetMode
                  //   SetSampleTime respectively

  // available but not commonly used functions ********************************************************
  void SetTunings(KPID); //   constructor, this function gives the user the option
                         //   of changing tunings during runtime for Adaptive control
  void SetTunings(KPID, int);


  // Display functions ****************************************************************
  KPID GetPID(); // These functions query the pid for interal values.
                 // they were created mainly for the pid front-end,
                 // where it's important to know what is actually

private:
  void Initialize();


  KPID tunings;

  int pOn;

  double *myInput;    // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;   //   This creates a hard link between the variables and the
  double *mySetpoint; //   PID, freeing the user from having to constantly tell us
                      //   what these values are.  with pointers we'll just know.

  // unsigned long lastTime;
  double outputSum, lastInput;

  unsigned int SampleTime;
  double outMin, outMax;
  bool pOnE;
};
#endif
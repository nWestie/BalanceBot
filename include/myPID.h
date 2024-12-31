#ifndef myPID
#define myPID

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

class PID {
public:
    // PID Constants
    struct KPID {
        double p;
        double i;
        double d;
    };

    // KPID, input, output, setpoint
    // constructor.  links the PID to the Input, Output, and
    // Setpoint.  Initial tuning parameters are also set here.
    // (overload for specifying proportional mode)
    PID(KPID, float *, float *, float *, unsigned int, bool);

    void Compute(); // * performs the PID calculation.  it should be
                    //   called every time loop() cycles. ON/OFF and
                    //   calculation frequency can be set using SetMode
                    //   SetSampleTime respectively

    // available but not commonly used functions ********************************************************
    void SetTunings(KPID); //   constructor, this function gives the user the option
                           //   of changing tunings during runtime for Adaptive control

    // KPID GetPID();

private:
    void Initialize();

    KPID tunings;

    float *myInput;    // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;   //   This creates a hard link between the variables and the
    float *mySetpoint; //   PID, freeing the user from having to constantly tell us
                       //   what these values are.  with pointers we'll just know.

    // unsigned long lastTime;
    float outputSum, lastInput;

    unsigned int SampleTime;
    float outMin, outMax;
    bool pOnE; // True if proportional on error, false for proportional on measurement
};
#endif
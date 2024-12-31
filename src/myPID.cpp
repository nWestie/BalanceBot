/*
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Updated for personal use by Noah West
 * This Library is licensed under the MIT License
 */

#include "Arduino.h"

#include <myPID.h>

/*Constructor (...)
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 */
PID::PID(KPID PidGains, unsigned int sampleTime_ms, bool POnError) {
    pOnE = POnError;

    outMin = -128; // default output limit corresponds to
    outMax = 127;  // the arduino pwm limits

    SampleTime = sampleTime_ms; // default Controller Sample Time is 0.1 seconds

    // PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(PidGains);

    // lastTime = millis() - SampleTime;
}

/* compute()
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 */
float PID::compute(float input, float setpoint) {
    // Compute all the working error variables
    float error = setpoint - input;
    float dInput = (input - lastInput); // change in input ()

    outputSum += (tunings.i * error); // integral term
    
    float output;
    if (pOnE)                       // Add Proportional on Error, if P_ON_E is specified
        output = tunings.p * error; // this is what I'll be using
    else                            // Add Proportional on Measurement, if P_ON_M is specified
    {
        output = 0;
        outputSum -= tunings.p * dInput;
    }

    // clamp integrator to output range(anti-windup)
    if (outputSum > outMax)
        outputSum = outMax;
    else if (outputSum < outMin)
        outputSum = outMin;

    /*Compute Rest of PID Output*/
    output += outputSum - tunings.d * dInput;

    // Clamp output
    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    lastInput = input;
    return output;
}

/* SetTunings(...)
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 */
void PID::SetTunings(KPID NewTunings) {
    // pOn = POn;
    // pOnE = POn == P_ON_E;

    float SampleTimeInSec = ((float)SampleTime) / 1000;
    tunings.p = NewTunings.p;
    tunings.i = NewTunings.i * SampleTimeInSec;
    tunings.d = NewTunings.d / SampleTimeInSec;
}

/* Initialize()
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 *  last input/output should be typical values for the PID controller, to prevent jerking on startup
 */
void PID::Initialize(float lastInput, float lastOutput) {
    outputSum = lastOutput;
    lastInput = lastInput;
    if (outputSum > outMax)
        outputSum = outMax;
    else if (outputSum < outMin)
        outputSum = outMin;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes. Will return time adjusted values, see Initialize()
 */
// PID::KPID PID::GetPID() { return tunings; }

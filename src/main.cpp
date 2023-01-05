#define DEBUG

#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <EEPROM.h>
#include "bt.h"

void waitForEnable(String);
void waitForEnable();
void updateBT(bool isEnabled);
void PIDupdate();
void PIDsave();

IMU imu;
float ypr[3];
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt = true;
}

#define LEDPIN 21

SoftTimer btUpdateTimer; // sends BT data at 20Hz

class Battery
{
public:
  Battery(uint8_t pin, void lowBattFunc(String str))
  {
    this->pin = pin;
    this->lowBattFunc = lowBattFunc;

    btInd = 0;
    battSmooth[0] = analogRead(pin) * battMult;
    for (int i = 1; i < 8; i++)
      battSmooth[i] = battSmooth[0];
  };
  double updateVoltage(bool isEnabled)
  {
    battSmooth[btInd] = analogRead(pin) * battMult;
    btInd++;
    btInd &= 0x07; // loops over above index 7 ('overflows')

    float battVoltAvg = 0;
    for (int i = 0; i < 8; i++)
      battVoltAvg += battSmooth[i];
    battVoltAvg /= 8;

    if (isEnabled && battVoltAvg < 11.18) //~20%. Don't call if disabled, to prevent call stack overflow
      lowBattFunc("LOW BATT");

    return battVoltAvg;
  };

private:
  const float battMult = (3.3 * 12.9) / (3 * 1024);
  uint8_t pin;
  float battSmooth[8];
  uint8_t btInd;
  void (*lowBattFunc)(String str);
};
Battery batt(20, waitForEnable);

class Motor
{
private:
  byte dirPin, pwmPin;

public:
  Encoder enc = Encoder(0, 0);
  Motor(byte pwm, byte dir, uint8_t e1, uint8_t e2)
  {
    dirPin = dir;
    pwmPin = pwm;
    enc = Encoder(e1, e2);
  }
  void drive(byte pwm, byte dir)
  {
    // digitalWrite(dirPin, dir);       **DISABLED FOR BT TESTING**
    // analogWrite(pwmPin, pwm);
    return;
  }
};
Motor lMotor(17, 15, 7, 8); // pwm, dir, enc1, enc2
Motor rMotor(16, 14, 5, 6);
void stopAll()
{
  lMotor.drive(0, 1);
  rMotor.drive(0, 1);
}

// Halts execution til restarted by controller
void waitForEnable();

double pitchDeg = 0;  // pitch in deg. From IMU, feedback for PID
double pitchSet = 90; // Combined controller and trim inputs, inp to PID
// double pitchInp = 90; // input from controller
// double pitchTrim = 0; // Trim value added to pitchSet. Manual correction for IMU error

double kPID[3] = {7, 42, .1};
double power = 0;
PID pidControl(&pitchDeg, &power, &pitchSet, kPID[0], kPID[1], kPID[2], REVERSE);

KivyBT bt(kPID, PIDupdate, PIDsave);
BTData btData = {0, 0, 0, false};

void setup()
{
  Serial.begin(38400);

  for (byte i = 5; i < 9; i++)
    pinMode(i, INPUT);
  for (byte i = 14; i < 18; i++)
    pinMode(i, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  btUpdateTimer.setTimeOutTime(50);
  btUpdateTimer.reset();

  kPID[0] = EEPROM.read(1);
  kPID[1] = EEPROM.read(2);
  kPID[2] = EEPROM.read(3);
  pidControl.SetTunings(kPID[0], kPID[1], kPID[2]);
  pidControl.SetOutputLimits(-255, 255);
  pidControl.SetSampleTime(10); // in ms
  bt.sendPID();

  attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);

  digitalWrite(LEDPIN, HIGH);

  bt.print("\nInitializing IMU");
  imu.setup(&pitchDeg);
  bt.print("IMU Setup Successful\n");
  bt.print("Press Power to Enable\n");
  IFD
  {
    while (Serial.read() == -1)
    {
      DPRINTLN("Waiting for Serial DBG");
      delay(500);
    }
    DPRINTLN("___End of Setup____");
  }
  waitForEnable();
}

void loop()
{
  // check IMU
  if (mpuInterrupt)
  {
    imu.update();                        // updates value of pitchDeg on interrupt
    if (pitchDeg < 40 || pitchDeg > 130) // halts if robot tips over
      waitForEnable();
  }

  // send BT data
  updateBT(true);
  if (!btData.enable)
    waitForEnable("Disabled by Controller");

  if (pidControl.Compute()) // updates at frequency given to PID controller
  {
    // update outputs based on pid, timed by PID lib
    // add steering
    // if (pitchInp > 90)
    // { // using joystick, not pid controller for if forward/backward
    //   l = power + steer;
    //   r = power - steer;
    // }
    // else
    // {
    //   l = power - steer;
    //   r = power + steer;
    // }
    int l, r;
    l = power;
    r = power;
    bool lDir = l > 0;
    bool rDir = r > 0;
    l = abs(l);
    r = abs(r);
    // send motor commands
    lMotor.drive(min(l, 255), lDir);
    rMotor.drive(min(r, 255), rDir);
  }
}
void waitForEnable(String message)
{
  bt.print("Disabled: ");
  bt.print(message);
  bt.print("\n");

  stopAll();

  pidControl.SetMode(MANUAL);

  bool flash = true;
  unsigned long blinkTime = millis() + 500;
  while (true)
  {
    if (mpuInterrupt)
    {
      imu.update();
    }
    if (millis() > blinkTime)
    {
      digitalWrite(LEDPIN, flash);
      flash = !flash;
      blinkTime = millis() + 500;
    }
    updateBT(false);
    if (btData.enable)
      break;
  }
  digitalWrite(LEDPIN, LOW);
  pidControl.SetMode(AUTOMATIC);
  bt.print("Enabled");
}
void waitForEnable()
{
  waitForEnable("");
}
void updateBT(bool isEnabled)
{
  if (btUpdateTimer.hasTimedOut())
  {
    btUpdateTimer.reset();

    double battVolt = batt.updateVoltage(isEnabled);
    bt.sendUpdate(battVolt, pitchSet, pitchDeg, isEnabled);

    bt.receiveData(&btData);
    pitchSet = 90 + (-btData.speed / 32) + btData.trim;
  }
}
void PIDupdate()
{
  DPRINT("PUpdate: ");
  DPRINT(kPID[0]);
  DPRINT(",");
  DPRINT(kPID[1]);
  DPRINT(",");
  DPRINTLN(kPID[2]);
  pidControl.SetTunings(kPID[0], kPID[1], kPID[2]);
};
void PIDsave()
{
  DPRINT("PSave: ");
  DPRINT(kPID[0]);
  DPRINT(",");
  DPRINT(kPID[1]);
  DPRINT(",");
  DPRINTLN(kPID[2]);
  EEPROM.write(1, kPID[0]);
  EEPROM.write(2, kPID[1]);
  EEPROM.write(3, kPID[2]);
  bt.print("PID Saved to EEPROM\n");
};
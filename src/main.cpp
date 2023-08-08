#define DEBUG

#include "Encoder.h"
#include "PID_v1.h"
#include "SoftTimers.h"
#include <EEPROM.h>

#include "debug.h"
#include "bt.h"
#include "IMU.h"
#include "bot.h"

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
private:
  const float battMult = (3.3 * 12.9) / (3 * 1024);
  uint8_t pin;
  float battSmooth[8];
  uint8_t btInd;
  void (*lowBattFunc)(String str);

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
};
Battery batt(20, waitForEnable);

class Motor
{
private:
  const byte dirPin, pwmPin;

public:
  Encoder enc = Encoder(0, 0);
  Motor(byte pwm, byte dir, uint8_t e1, uint8_t e2) : dirPin(dir), pwmPin(pwm)
  {
    enc = Encoder(e1, e2);
  }
  void drive(byte pwm, byte dir)
  {
    digitalWriteFast(dirPin, dir);
    analogWrite(pwmPin, pwm);
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
double pitchSet = 90; // Combined controller and trim inputs, setpoint for PID
double power = 0;     // Motor power, output from PID.


BTInterface *bt = &KivyBT(PIDupdate, PIDsave);
BTData btData = {0, 0, 0, false};
Bot bot = Bot();

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

  attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);

  digitalWriteFast(LEDPIN, HIGH);

  bt->print("\nInitializing IMU");
  imu.setup(&pitchDeg);
  bt->print("IMU Setup Successful\n");
  bt->print("Press Power to Enable\n");
  // IFD
  // {
  //   while (Serial.read() == -1)
  //   {
  //     DPRINTLN("Waiting for Serial DBG");
  //     delay(500);
  //   }
  //   DPRINTLN("___End of Setup____");
  // }

  waitForEnable();
}

void loop()
{
  // check IMU
  if (mpuInterrupt)
  {
    imu.update();                        // updates value of pitchDeg on interrupt
    if (pitchDeg < 40 || pitchDeg > 130) // halts if robot tips over
      waitForEnable("Angle Out of Bounds");
  }

  // send BT data
  updateBT(true);
  if (!btData.enable)
    waitForEnable("Disabled by Controller");
  else if (!bt->connected)
    waitForEnable("Connection Lost");
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
  bool disableAcknowledged = false; // wait for controller to concur that the bot is disabled before accepting an enable
  bt->print("Disabled: ");
  bt->print(message);
  bt->print("\n");

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
    if (!disableAcknowledged && !btData.enable)
    {
      disableAcknowledged = true;
      DPRINTLN("Lock Disable");
    }
    else if (btData.enable && disableAcknowledged)
    {
      break;
    }
  }
  DPRINTLN("Enabling");
  digitalWrite(LEDPIN, LOW);
  pidControl.SetMode(AUTOMATIC);
  bt.print("Enabled\n");
}
void waitForEnable()
{
  waitForEnable("");
}
// Send BT logging updates, and recieve and parse control. Saves control into btData struct, and pitchSet(input to PID)
void updateBT(bool isEnabled)
{
  if (btUpdateTimer.hasTimedOut())
  {
    btUpdateTimer.reset();

    double battVolt = batt.updateVoltage(isEnabled);
    bt.sendUpdate(battVolt, pitchSet, pitchDeg, isEnabled);

    bt.receiveData(&btData);
    pitchSet = 90 + (btData.speed / 32) + btData.trim;
  }
}
void PIDupdate()
{
  DPRINT("PUpdate: ");
  DPRINT(pid[0]);
  DPRINT(",");
  DPRINT(pid[1]);
  DPRINT(",");
  DPRINTLN(pid[2]);
  pidControl.SetTunings(pid[0], pid[1], pid[2]);
};
void PIDsave()
{
  DPRINT("PSave: ");
  DPRINT(pid[0]);
  DPRINT(",");
  DPRINT(pid[1]);
  DPRINT(",");
  DPRINTLN(pid[2]);
  EEPROM.write(1, pid[0]);
  EEPROM.write(2, pid[1]);
  EEPROM.write(3, pid[2]);
  bt.print("PID Saved to EEPROM\n");
};
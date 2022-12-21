//#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <Stream.h>
#include <EEPROM.h>

#define RPRINTLN(s)    \
  Serial2.print("*T"); \
  Serial2.print(s);    \
  Serial2.print("\n*")

constexpr float BATTMULT = (3.3 * 12.9) / (3 * 1024);
#define BTPIN 20
#define BATSMOOTHSIZE 7
float battSmooth[BATSMOOTHSIZE];
byte btInd = 0;
float battVolt;

IMU imu;
float ypr[3];

bool enable = false;

#define LEDPIN 21

SoftTimer battRefreshTimer, timeOut;

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
int steer;
void stopAll()
{
  lMotor.drive(0, 1);
  rMotor.drive(0, 1);
}

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
// Halts execution til restarted by controller
void waitForEnable();
//  
void sendData();
void checkInput();

double pitchDeg = 0; // pitch in deg. From IMU, feedback for PID
double pitchSet = 90; // Combined controller and trim inputs, inp to PID
double pitchInp = 90; // input from controller
double pitchTrim = 0; // Trim value added to pitchSet. Manual correction for IMU error
double power;

double kpPitch = 7, kiPitch = 42, kdPitch = .1;
PID pidPitch(&pitchDeg, &power, &pitchSet, kpPitch, kiPitch, kdPitch, REVERSE);
void setup()
{
  for (byte i = 5; i < 9; i++)
    pinMode(i, INPUT);
  for (byte i = 14; i < 18; i++)
    pinMode(i, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(38400);
  Serial2.begin(38400);

  battRefreshTimer.setTimeOutTime(200);
  battRefreshTimer.reset();
  timeOut.setTimeOutTime(400);
  timeOut.reset();

  kpPitch = EEPROM.read(1);
  kiPitch = EEPROM.read(2);
  kdPitch = EEPROM.read(3);
  pidPitch.SetTunings(kpPitch, kiPitch, kdPitch);
  
  pidPitch.SetOutputLimits(-255, 255);
  pidPitch.SetSampleTime(10);
  attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);

  digitalWrite(LEDPIN, HIGH);

  for (int i = 0; i < 7; i++)
    battSmooth[i] = analogRead(BTPIN);

  RPRINTLN("\nSetting Up IMU");
  imu.setup(&pitchDeg);
  Serial2.print("*TIMU Setup Successful\n*");
  pidPitch.SetMode(AUTOMATIC);
  Serial2.print("*TPID Controller Started\n*");
  Serial2.print("*TPress Power to Enable\n*");
  waitForEnable();
}
int l, r;

void loop()
{
  if (mpuInterrupt)
  {
    imu.update(); // updates value of pitchDeg on interrupt
    if (pitchDeg < 40 || pitchDeg > 130)
    {
      stopAll();
      waitForEnable();
    }
  }

  checkInput();  // get new inputs
  sendData(); // send data if needed

  if (pidPitch.Compute()){ 
    // update outputs based on pid, timed by PID lib
    // add steering
    if (pitchInp > 90){ // using joystick, not pid controller for if forward/backward
      l = power + steer;
      r = power - steer;
    }
    else{
      l = power - steer;
      r = power + steer;
    }
    bool lDir = l > 0;
    bool rDir = r > 0;
    l = abs(l);
    r = abs(r);
    // send motor commands
    lMotor.drive(min(l, 255), lDir);
    rMotor.drive(min(r, 255), rDir);
  }
}
void waitForEnable(){
  RPRINTLN("Disabled");
  enable = false;
  bool flash = true;
  pidPitch.SetMode(MANUAL);
  unsigned long t = millis() + 500;
  while (true){
    if (enable){
      break;
    }
    if (mpuInterrupt){
      imu.update();
    }
    sendData();
    checkInput();
    if (millis() > t){
      digitalWrite(LEDPIN, flash);
      flash = !flash;
      t = millis() + 500;
    }
  }
  digitalWrite(LEDPIN, LOW);
  timeOut.reset();
  pidPitch.SetMode(AUTOMATIC);
  RPRINTLN("Enabled");
  enable = true;
};
void sendData(){
  if (battRefreshTimer.hasTimedOut()){
    battRefreshTimer.reset();

    battSmooth[btInd] = analogRead(BTPIN) * BATTMULT;
    btInd = (btInd + 1) % BATSMOOTHSIZE;

    battVolt = 0;
    for (int i = 0; i < BATSMOOTHSIZE; i++)
      battVolt += battSmooth[i];
    battVolt /= BATSMOOTHSIZE;

    if (battVolt < 11.15)
    {
      Serial2.print("*\nTLOW BATT\nStopping motors\n*");
      stopAll();
      waitForEnable();
    }

    Serial2.print("*V");
    Serial2.print(battVolt);
    Serial2.print("*");
    Serial2.print(":");
    Serial2.print(",");
    Serial2.print(pitchSet);
    Serial2.print(",");
    Serial2.print(pitchDeg);
    Serial2.print(":");
  }
};
void checkInput(){
  while (Serial2.available()){
    // switch based on which slider its from
    char sw;
    sw = Serial2.read(); // read ID
    switch (sw){
    case 'X':
      steer = Serial2.parseInt();
      steer -= 255;
      steer /= 8;
      break;
    case 'Y':
    {
      int mPow = Serial2.parseInt();
      mPow -= 255;
      mPow = -mPow;
      pitchInp = 90 + (mPow / 32); // conversion to setpoint
      pitchSet = pitchInp + pitchTrim;
      break;
    }
    case 'S':
      if (enable){
        stopAll();
        waitForEnable();
      }
      else
        enable = true;
      break;
    case 'F':
      pitchTrim += .1;
      pitchSet = pitchInp + pitchTrim;
      break;
    case 'B':
      pitchTrim -= .1;
      pitchSet = pitchInp + pitchTrim;
      break;
    case 'M':
    { // terminal commands
      while (!Serial2.available())
        ;
      char sw = Serial2.read();
      if (isLowerCase(sw))
      {
        sw -= 32;
      }
      double newVal = Serial2.parseFloat();
      switch (sw)
      {
      case 'P':
        kpPitch = newVal;
        break;
      case 'I':
        kiPitch = newVal;
        break;
      case 'D':
        kdPitch = newVal;
        break;
      case 'S':
        EEPROM.write(1,kpPitch);
        EEPROM.write(2,kiPitch);
        EEPROM.write(3,kdPitch);
        Serial2.print("*TSaved *");
      }
      pidPitch.SetTunings(kpPitch, kiPitch, kdPitch);
      Serial2.print("*TPID: ");
      Serial2.print(pidPitch.GetKp());
      Serial2.print(", ");
      Serial2.print(pidPitch.GetKi());
      Serial2.print(", ");
      Serial2.print(pidPitch.GetKd());
      Serial2.print("\n*");
      break;
    }
    case '/':
      break; // End character
    }
  }
}
//#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <Stream.h>

#define RPRINTLN(s) Serial2.print("*T");Serial2.print(s);Serial2.print("\n*")
bool enable = false;
constexpr float BATTMULT = (3.3*12.9)/(3*1024);
#define BTPIN 20
#define BATSMOOTHSIZE 7
float battSmooth[BATSMOOTHSIZE];
byte btInd = 0;
float battVolt;

IMU imu;
float ypr[3];

#define LEDPIN 21

SoftTimer battRefreshTimer, timeOut;

class Motor{
private:
  byte dirPin, pwmPin;
public:
  Encoder enc = Encoder(0,0);
  Motor(byte pwm, byte dir, uint8_t e1, uint8_t e2) {
    dirPin = dir; 
    pwmPin = pwm;
    enc = Encoder(e1, e2);
  }
  void drive(byte pwm, byte dir){
    digitalWrite(dirPin, dir);
    analogWrite(pwmPin, pwm);
    return;
  }
};
Motor lMotor(17, 15, 7, 8); //pwm, dir, enc1, enc2
Motor rMotor(16, 14, 5, 6);
int steer;
void stopAll(){
  lMotor.drive(0,1);
  rMotor.drive(0,1);
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void waitForEnable();
void checkBattSend();
void checkInput();


//pitchDeg: degrees pitch, output from imu, input to pid
double pitchDeg = 0 , pitchSet = 90;
double pitchInp, pitchOffset = 0; //pitch inp is from controller, pitchOffset is added to it and saved to pitchSet. Corrects for IMU error
double power;

double kp = 12, ki = 42, kd = .4;
PID pidMain(&pitchDeg, &power, &pitchSet, kp, ki, kd, REVERSE);

void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(38400);
  Serial2.begin(38400);

  battRefreshTimer.setTimeOutTime(200);
  battRefreshTimer.reset();
  timeOut.setTimeOutTime(400);
  timeOut.reset();
  
  pidMain.SetOutputLimits(-255, 255);
  pidMain.SetSampleTime(10);
  attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);

  digitalWrite(LEDPIN, HIGH);

  for(int i = 0; i < 7; i++) battSmooth[i] = analogRead(BTPIN);

 
  RPRINTLN("\nSetting Up IMU");  
  imu.setup(&pitchDeg);
  Serial2.print("*TIMU Setup Successful\n*"); 
  pidMain.SetMode(AUTOMATIC);
  Serial2.print("*TPID Controller Started\n*"); 
  Serial2.print("*TPress Power to Enable\n*"); 
  waitForEnable();
}
int l;
int r;

void loop(){
  checkBattSend();
  if(mpuInterrupt){
    imu.update(); //updates value of pitchDeg on interrupt
    if(pitchDeg<40||pitchDeg>130){
      stopAll();
      waitForEnable();
    }
  }
  checkInput();  

  if(pidMain.Compute()){ //update outputs based on pid, timed by PID lib
    //add steering
    l = power + steer;
    r = power - steer;
    bool lDir = l>0;
    bool rDir = r>0;
    l = abs(l);
    r = abs(r);
    //send motor commands
    lMotor.drive(min(l, 255), lDir);
    rMotor.drive(min(r,255), rDir);
  }
}
void waitForEnable(){
  RPRINTLN("Disabled");
  enable = false;
  bool flash = true;
  pidMain.SetMode(MANUAL);
  unsigned long t = millis()+500;
  while(true){
    if(enable)break;
    checkBattSend();
    checkInput();
    if(millis()>t){
      digitalWrite(LEDPIN, flash);
      flash = !flash;
      t = millis()+500;
    }
  }
  digitalWrite(LEDPIN, LOW);
  timeOut.reset();
  pidMain.SetMode(AUTOMATIC);
  RPRINTLN("Enabled");
  enable = true;
};
void checkBattSend(){
  if(battRefreshTimer.hasTimedOut()){
      battRefreshTimer.reset();

      battSmooth[btInd] = analogRead(BTPIN)*BATTMULT;
      btInd = (btInd + 1)%BATSMOOTHSIZE;
      
      battVolt=0;
      for(int i = 0; i < BATSMOOTHSIZE; i++)battVolt+=battSmooth[i];
      battVolt /= BATSMOOTHSIZE;
      
      Serial2.print("*V");
      Serial2.print(battVolt);
      Serial2.print("*");
      Serial2.print("*C");
      Serial2.print(battVolt);
      Serial2.print("*");
      if(battVolt <11.15){
      Serial2.print("*\nTLOW BATT\nStopping motors\n*");
      stopAll();
      waitForEnable(); 
      }

      Serial2.print("*G");
      Serial2.print(pitchSet);  
      Serial2.print(",");
      Serial2.print(pitchDeg);
      Serial2.print("*");
        
  } 
};
void checkInput(){
  while(Serial2.available()){
    //switch based on which slider its from
    char sw;
    sw = Serial2.read(); //read ID
    switch(sw){
    case 'X':
      steer = Serial2.parseInt();
      steer -= 255;
      steer /= 8;
      break;
    case 'Y':{
      int mPow = Serial2.parseInt();
      mPow -= 255;
      mPow = -mPow;
      pitchInp = 90 + (mPow / 32); //conversion to setpoint, just a variable +- 8 deg for now
      pitchSet = pitchInp+pitchOffset;
      break;
    }
    case 'S':
      if(enable){
        stopAll();
        waitForEnable();
      }else enable = true;
      break;
    case 'F':
      pitchOffset += .2;
      pitchSet = pitchInp+pitchOffset;
      break;
    case 'B':
      pitchOffset -= .2;
      pitchSet = pitchInp+pitchOffset;
      break;
    case 'M': {//terminal commands
      while(!Serial2.available());
      char sw = Serial2.read();
      if(isLowerCase(sw)){
        sw -= 32;
      }
      double newVal = Serial2.parseFloat();
      switch (sw){
      case 'P':
        kp = newVal;
        break;
      case 'I':
        ki = newVal;
        break;
      case 'D':
        kd = newVal;
        break;
      }
      pidMain.SetTunings(kp, ki, kd);
      Serial2.print("*TPID: ");
      Serial2.print(pidMain.GetKp());
      Serial2.print(", ");
      Serial2.print(pidMain.GetKi());
      Serial2.print(", ");
      Serial2.print(pidMain.GetKd());
      Serial2.print("\n*");
      break;
      }
    case '/':
      break; // End character
    }
  }
}
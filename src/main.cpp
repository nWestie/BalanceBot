//#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <Stream.h>

#define RPRINTLN(s) Serial2.print("*T");Serial2.print(s);Serial2.print("\n*")

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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void stopAll(){
  lMotor.drive(0,1);
  rMotor.drive(0,1);
}
void waitForEnable(String message);
void checkBattSend();
void checkInput();

//pitchDeg: degrees pitch, output from imu, input to pid
double pitchDeg = 0 , pitchSet = 90;
double power;

double kp = 8, ki = 5, kd = .2;
PID pidMain(&pitchDeg, &power, &pitchSet, kp, ki, kd, REVERSE);

void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(9600);

  battRefreshTimer.setTimeOutTime(200);
  battRefreshTimer.reset();
  timeOut.setTimeOutTime(400);
  timeOut.reset();
  
  pidMain.SetOutputLimits(-255, 255);
  
  attachInterrupt(digitalPinToInterrupt(22), dmpDataReady, RISING);

  digitalWrite(LEDPIN, HIGH);

  for(int i = 0; i < 7; i++) battSmooth[i] = analogRead(BTPIN);

 
  RPRINTLN("\nSetting Up IMU");  
  imu.setup(&pitchDeg);
  Serial2.print("*TIMU Setup Successful\n*"); 
  pidMain.SetMode(AUTOMATIC);
  Serial2.print("*TPID Controller Started\n*"); 
  Serial2.print("*TPress Power to Enable\n*"); 
  waitForEnable("");
}
int l;
int r;

void loop(){
  checkBattSend();
  if(mpuInterrupt){
    imu.update(); //updates value of pitchDeg on interrupt
  }
  checkInput();  

  if(pidMain.Compute()){ //update outputs based on pid, timed by PID lib
    // Serial.println("PID/output called");
    // Serial.print("Update control, Power: ");
    // Serial.print(power);
    // Serial.print("Setpoint: ");
    // Serial.print(pitchSet);
    // Serial.print("Angle: ");
    // Serial.println(pitchDeg);
    //add steering
    l = power; // + steer;
    r = power; //- steer;
    bool lDir = l>0;
    bool rDir = r>0;
    l = abs(l);
    r = abs(r);
    //send motor commands
    // lMotor.drive(min(l, 255), lDir);
    // rMotor.drive(min(r,255), rDir);
  }
  if(timeOut.hasTimedOut()){
    stopAll();
    RPRINTLN("Missed Heartbeat\nPress power to reenable");
    waitForEnable("");
  }
}
void waitForEnable(String message){
  bool en = true;
  pidMain.SetMode(MANUAL);
  unsigned long t = millis()+500;
  while(true){
    if(Serial2.read()=='P')break;
    checkBattSend();
    checkInput();
    if(millis()>t){
      digitalWrite(LEDPIN, en);
      en = !en;
      t = millis()+500;
      //if(message.length())Serial2.print("*T\n\n\n\n\n\n" + message + "\n*");  
    }
    //delay(20);
  }
  digitalWrite(LEDPIN, LOW);
  timeOut.reset();
  pidMain.SetMode(AUTOMATIC);
  Serial.println("Ending waitForEnable");
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
      waitForEnable(""); 
      }  
  } 
};
void checkInput(){
  while(Serial2.available()){
    //switch based on which slider its from
    char sw;
    sw = Serial2.read(); //read ID
    Serial.print(sw);
    switch(sw){
    case 'X':
      steer = Serial2.parseInt();
      steer -= 255;
      steer /= 2;
      Serial.print("Steer");
      break;
    case 'Y':
      int mPow = Serial2.parseInt();
      mPow -= 255;
      mPow = -mPow;
      pitchSet = 90+ (mPow / 64); //conversion to setpoint, just a variable +- 8 deg for now
      Serial.print("Power");
      break;
    case 'P':
      timeOut.reset();
      Serial.print("Heartbeat");
      break;
    case 'p':
      Serial.print("kill");
      stopAll();
      waitForEnable("Press Power to reenable");
      break;
    case 'M': {//terminal commands
      Serial.print("terminal");
      while(true){
        char sw = Serial2.read();
        sw = sw & 0xDF; //to uppercase
        Serial.print(String(sw));
        if(!isUpperCase(sw)){
          RPRINTLN("Invalid Input");
          break;
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
      }
      pidMain.SetTunings(kp, ki, kd);
      Serial2.print("*t\nPID: ");
      Serial2.print(pidMain.GetKd());
      Serial2.print(", ");
      Serial2.print(pidMain.GetKi());
      Serial2.print(", ");
      Serial2.print(pidMain.GetKp());
      Serial2.print("\n*");
      break;
      }
    case '/':
      break; // End character
    }
  }
}
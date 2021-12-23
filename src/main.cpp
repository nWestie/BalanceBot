//#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <Stream.h>

constexpr float BATTMULT = (3.3*12.9)/(3*1024);
#define BTPIN 20
#define BATSMOOTHSIZE 7
float battSmooth[BATSMOOTHSIZE];
byte btInd = 0;
float battVolt;

IMU imu;
float ypr[3];

#define LEDPIN 21

SoftTimer mainLoopTimer, battRefreshTimer, timeOut;

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
int power, steer;

void stopAll(){
  lMotor.drive(0,1);
  rMotor.drive(0,1);
}
void waitForEnable();

void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);

  Serial.begin(9600);
  Serial2.begin(9600);

  mainLoopTimer.setTimeOutTime(40);
  mainLoopTimer.reset();
  battRefreshTimer.setTimeOutTime(200);
  battRefreshTimer.reset();
  timeOut.setTimeOutTime(400);
  timeOut.reset();
  
  Serial2.print("*TPress Power to Initialize\n IMU and Enable Motors\n*");  
  //waitForEnable();

  Serial2.print("*TSetting Up IMU\n*");  
  // imu.setup();
  Serial2.print("*TIMU Setup Successful\n*");  
}

void loop(){
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
  }
  //actually run loop every 40 ms
  if(mainLoopTimer.hasTimedOut()){
    mainLoopTimer.reset(); 
    while(Serial2.available()){
      char sw;
      sw = Serial2.read(); //read ID
      //switch based on which slider its from
      switch(sw){
      case 'J': {
        while(true){
          if (Serial2.available()){
            char inp = Serial2.read();  //Get next character 
            if(inp=='X') steer = Serial2.parseInt();
            if(inp=='Y') power =Serial2.parseInt();
            if(inp=='/') break; // End character
          }
        }

        power -= 255;
        power = -power;
        steer -= 255;
        if(abs(power)<20) power = 0;
        if(abs(steer)<20) steer = 0;
        
        //PID stuff will go here
        
        //add steering
        byte l = power-steer;
        byte r = power+steer;

        Serial.print(power);
        Serial.print(", ");
        Serial.print(steer);
        Serial.print(", ");
        Serial.print(l);
        Serial.print(", ");
        Serial.println(r);

        lMotor.drive(min(abs(l), 255), l>0);
        rMotor.drive(min(abs(r),255), r>0);
        break;
      }
      case 'P':
        timeOut.reset();
        break;
      case 'p':
        stopAll();
        waitForEnable();
        break;
      }
    }
  }
  // if(timeOut.hasTimedOut()){
  //   stopAll();
  //   waitForEnable();
  // }
}
void waitForEnable(){
  bool en = true;
  SoftTimer ledDel;
  ledDel.setTimeOutTime(500);
  while(true){
    if(Serial2.read()=='P')break;
    if(ledDel.hasTimedOut()){
      digitalWrite(LEDPIN, en);
      en = !en;
    }
    delay(20);
  }
};

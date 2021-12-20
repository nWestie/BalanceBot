//#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
#include "SoftTimers.h"
#include <Stream.h>

constexpr float BATTMULT = (3.3*12.9)/(3*1024);
#define BTPIN 20
#define BATSMOOTHSIZE 5
float battSmooth[BATSMOOTHSIZE];
byte btInd = 0;


IMU imu;

SoftTimer mainLoopTimer, battRefreshTimer;
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
float ypr[3];
Motor lMotor(17, 15, 7, 8); //pwm, dir, enc1, enc2
Motor rMotor(16, 14, 5, 6);
void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);

  Serial.begin(9600);
  Serial2.begin(9600);

  mainLoopTimer.setTimeOutTime(40);
  mainLoopTimer.reset();
  battRefreshTimer.setTimeOutTime(200);
  battRefreshTimer.reset();
  
  IFD while (Serial.available() && Serial.read()); // empty buffer
  IFD while (!Serial.available());                 // wait for data
  IFD while (Serial.available() && Serial.read()); // empty buffer again

  //imu.setup();
  IFD Serial.println("IMU Setup Successful");  
  Serial2.print("*TIMU Setup Successful\n*");  
}
float battVolt;
void loop(){
if(battRefreshTimer.hasTimedOut()){
  battRefreshTimer.reset();
  battSmooth[btInd] = analogRead(BTPIN)*BATTMULT;
  btInd = (btInd + 1)%BATSMOOTHSIZE;
  battVolt=0;
  for(int i = 0; i < BATSMOOTHSIZE; i++)battVolt+=battSmooth[i];
  battVolt /= 5;
  Serial2.print("*V");
  Serial2.print(battVolt);
  Serial2.print("*");
  Serial2.print("*C");
  Serial2.print(battVolt);
  Serial2.print("*");
}
//actually run loop every 20 ms
if(mainLoopTimer.hasTimedOut()){
  mainLoopTimer.reset(); 
  
  
  if(Serial2.available()){
    char sw;
    int val;
    bool dir = true;

    sw = Serial2.read(); //read ID
    val = Serial2.parseInt(); //read num
    Serial2.read(); // consume '/'
    val = map(val, 0, 100, -255, 255);
    //remap to 0-255 and dir
    if(val<0) {
      dir = false;
      val = -val;
    }
    if(val<30) val = 0;
    val/=2;
    //switch based on which slider its from
    switch(sw){
    case 'A':
      lMotor.drive(val, dir);
      break;
    case 'B':
      rMotor.drive(val, dir);
      break;
    }
    IFD Serial.printf("%c, %d", sw, val);
  }
}}

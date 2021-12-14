#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
IMU imu;
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
//Motor lMotor(17, 15, 7, 8); //pwm, dir, enc1, enc2
//Motor rMotor(16, 14, 5, 6);
void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(9600);
  IFD while (Serial.available() && Serial.read()); // empty buffer
  IFD while (!Serial.available());                 // wait for data
  IFD while (Serial.available() && Serial.read()); // empty buffer again

  //imu.setup();
  Serial.println("IMU Setup Successful");  
}

void loop(){
  //send any software serial data back out on serial link
  if(Serial2.available()){
    char inp[3];
    Serial2.readBytes(inp, 4);
    for(byte i = 0;i<4;i++)Serial.print(inp[i], 16);
    Serial.println();
  }
}

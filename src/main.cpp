//#define DEBUG
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
Motor lMotor(17, 15, 7, 8); //pwm, dir, enc1, enc2
Motor rMotor(16, 14, 5, 6);
void setup(){
  for(byte i = 5; i < 9; i++) pinMode(i, INPUT);
  for(byte i = 14; i < 18; i++) pinMode(i, OUTPUT);
  
  IFD while (Serial.available() && Serial.read()); // empty buffer
  IFD while (!Serial.available());                 // wait for data
  IFD while (Serial.available() && Serial.read()); // empty buffer again

  imu.setup();
  Serial.println("IMU Setup Successful");
  Serial.println(lMotor.enc.read());
}
bool alt = false;
void loop(){
  if(alt)lMotor.drive(255, 1);
  else lMotor.drive(0,1);
  Serial.println(alt);
  delay(1000);
  alt = !alt;
  // if(imu.update(ypr)){
  //   Serial.print("ypr\t");
  //   Serial.print(ypr[0] * 180/M_PI);
  //   Serial.print("\t");
  //   Serial.print(ypr[1] * 180/M_PI);
  //   Serial.print("\t");
  //   Serial.println(ypr[2] * 180/M_PI);
  // }
}

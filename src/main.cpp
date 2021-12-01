#define DEBUG
#include "debug.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "IMU.h"
IMU imu;
class Motor{
public:
  Encoder enc = Encoder(0,0);
  Motor(uint8_t pwm, uint8_t dir, uint8_t e1, uint8_t e2) {
    dirPin = dir; pwmPin = pwm;
    enc = Encoder(e1, e2);
  }
  void drive(int8_t pwm, bool dir){
    digitalWrite(dirPin, dir);
    analogWrite(pwmPin, pwm);
  }
private:
  int8_t dirPin, pwmPin;
};
float ypr[3];
void setup(){
  IFD while (Serial.available() && Serial.read()); // empty buffer
  IFD while (!Serial.available());                 // wait for data
  IFD while (Serial.available() && Serial.read()); // empty buffer again

  imu.setup();
  Serial.println("IMU Setup Successful");
  Motor lMotor(17, 15, 7, 8);
  Motor rMotor(16, 14, 5, 6);
  Serial.println(lMotor.enc.read());

}
void loop(){
  while (Serial.available() && Serial.read());
  while (!Serial.available());                
  while (Serial.available() && Serial.read());
  if(imu.update(ypr)){
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
}

#include <Arduino.h>
#include <Encoder.h>
#include <IMU.h>
//encoder defines
#define ENCODER_OPTIMIZE_INTERRUPTS


Encoder m2(7, 8);
const float BATTMULT = (3.3*13)/(3*1024);
IMU imu;
void setup(){
  imu.init();
  Serial.begin(9600);
  pinMode(15, OUTPUT); //m1 dir
  pinMode(17, OUTPUT); //m1 pwm
  pinMode(20, INPUT); //Batt Monitering

}
void loop(){
  //Serial.println(analogRead(20)*BATTMULT);
  while(imu.update());
  delay(100);
  
}
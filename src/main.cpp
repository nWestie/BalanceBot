#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Encoder.h>
#include "IMU.h"
//encoder defines


Encoder m2(7, 8);
const float BATTMULT = (3.3*13)/(3*1024);
IMU imu;
void setup(){
  Serial.begin(9600);
  
  imu.init();
  pinMode(15, OUTPUT); //m1 dir
  pinMode(17, OUTPUT); //m1 pwm
  pinMode(20, INPUT); //Batt Monitering

}
void loop(){
  Serial.println(analogRead(20)*BATTMULT);
  imu.update();
  delay(100);
  /*Serial.println(m2.read());
  digitalWrite(15, HIGH);
  digitalWrite(17, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
  Serial.println(m2.read());
  delay(1000);
  Serial.println(m2.read());
  digitalWrite(15, LOW);
  digitalWrite(17, HIGH);
  delay(1000); 
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
  Serial.println(m2.read());
  delay(1000);//*/
}



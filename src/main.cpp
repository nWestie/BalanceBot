#include <Arduino.h>
#include <Encoder.h>
#include <MPU6050.h>
//#define ENCODER_OPTIMIZE_INTERRUPTS
Encoder m2(7, 8);
const float BATTMULT = (3.3*13)/(3*1024);

void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(15, OUTPUT); //m1 dir
  pinMode(17, OUTPUT); //m1 pwm
  digitalWrite(13, HIGH);
  pinMode(20, INPUT);
}
void loop(){
  Serial.println(analogRead(20)*BATTMULT);
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
  delay(1000);*/
}
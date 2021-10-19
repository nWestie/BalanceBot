#include <Arduino.h>
#include <Encoder.h>
//#define ENCODER_OPTIMIZE_INTERRUPTS

Encoder m2(7, 8);
void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(15, OUTPUT); //m1 dir
  pinMode(17, OUTPUT); //m1 pwm
  digitalWrite(13, HIGH);
}
void loop(){
  Serial.println(m2.read());
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
  delay(1000);
}
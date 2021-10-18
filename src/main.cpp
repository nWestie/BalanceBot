#include <Arduino.h>
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

Encoder m1(5, 6);
void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}
void loop(){
  Serial.println(m1.read());
  delay(100);
}
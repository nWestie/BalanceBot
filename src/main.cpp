//#define DEBUG
#include <Arduino.h>
void setup(){
  Serial.begin(9600);
  Serial2.begin(9600);
}
void loop(){
  if(Serial.available())Serial2.write(Serial.read());
  if(Serial2.available())Serial.write(Serial2.read());
}

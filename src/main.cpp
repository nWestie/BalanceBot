#define DEBUG
#include "debug.h"

#include "IMU.h"
IMU imu;
float ypr[3];
void setup(){
  IFD while (Serial.available() && Serial.read()); // empty buffer
  IFD while (!Serial.available());                 // wait for data
  IFD while (Serial.available() && Serial.read()); // empty buffer again
  imu.setup();
  DPRINTLN("Test debug print in main");
}
void loop(){
  if(imu.update(ypr)){
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
}

#include <helper_3dmath.h>
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_9Axis_MotionApps41.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  Serial2.begin(9600);
  while(!Serial2);
  digitalWrite(21, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

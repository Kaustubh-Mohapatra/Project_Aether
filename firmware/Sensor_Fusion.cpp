// Trying to fuse gyrometer and accelerometer with this one to try simulate rudder
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Servo rudder;

void setup() {
  Serial.begin(115200);
  rudder.attach();
  Wire.begin();

}

void loop() {

}

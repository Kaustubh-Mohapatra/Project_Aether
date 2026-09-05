#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Servo rservo;
Servo cservo;
Servo lservo;

void setup() {
    Serial.begin(115200);
    Wire.begin(21,22);
    rservo.attach(25);
    lservo.attach(33);
    cservo.attach(32);

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    mpu.getEvent(&a, &g, &temp);

    float roll = atan2(-a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x +a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y +a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    // Sadly cant add yaw due to limitations to my sensors_event

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Z axis rotation speed: "); // Can only check gyration speed so later can add it to integration part of pid loop
    Serial.println(g.gyro.z);

    int servoAngleP = 90 - pitch;
    servoAngleP = constrain(servoAngleP, 20, 160);

    int servoAngleR = 90 - roll;
    servoAngleR = constrain(servoAngleR, 20, 160);

    rservo.write(servoAngleR);    // Mechanically
    lservo.write(servoAngleR);    // Reversed
    cservo.write(servoAngleP);

}

#include <Arduino.h>
#include <ESP32Servo.h>   // Servo lib for esp32

Servo servo;

void setup() {
    servo.attach(25);
}

void loop() {

    servo.writeMicroseconds(0); // 0 deg
    delay(1000);

    servo.writeMicroseconds(500); // 0 deg
    delay(1000);

    servo.writeMicroseconds(1000); // 45 deg
    delay(1000);

    servo.writeMicroseconds(1500); // 90 deg
    delay(1000);

    servo.writeMicroseconds(2000); // 135 deg
    delay(1000);

    servo.writeMicroseconds(2500); // 180 deg
    delay(1000);

    // Better to use microseconds as degrees need not be universally the same but microseconds will definately by because of how PWM works




}

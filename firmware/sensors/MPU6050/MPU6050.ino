#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(){
    Serial.begin(115200);
    while (!Serial)
        delay(10);  // Paused till serial monitor opened

    Serial.println("MPU6050 test");

    if (!mpu.begin()) {     // Try initializing mpu
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
        delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU_RANGE_8_G);   // Print Accelerometer range of MPU
    Serial.print("Accelerometer range set to: ");
    switch (mpu.setAccelerometerRange()){
        case MPU_RANGE_2_G:
        Serial.println("+-2G");
        break;
        case MPU_RANGE_4_G:
        Serial.println("+-4G");
        break;
        case MPU_RANGE_8_G:
        Serial.println("+-8G");
        break;
        case MPU_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    
}

void loop(){

}
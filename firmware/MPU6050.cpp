#include <Arduino.h>
#include <Adafruit_MPU6050.h>   // Lib for MPU6050
#include <Adafruit_Sensor.h>    // Lib for Sensors in general
#include <Wire.h>               // Lib for I²C protocol

Adafruit_MPU6050 mpu;           // Initialize the device

void setup(){
    Serial.begin(115200);

    Wire.begin(21, 22);

    Serial.println("MPU6050 test");

    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("MPU initialization check failed");
        while (1){                                  // Stops the running of code if MPU doesnt get detected so we dont end up messing our other stuff up
            delay(10);
        }
    }
    else {
        Serial.println("MPU initialization successful");
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // Sets Accelerometer range of MPU
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()){
        case MPU6050_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case MPU6050_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case MPU6050_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case MPU6050_RANGE_16_G:
            Serial.println("+-16G");
            break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);    // Sets the Gyro range of MPU
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()){;
        case MPU6050_RANGE_250_DEG:
            Serial.println("+-250 deg/s");
            break;
        case MPU6050_RANGE_500_DEG:
            Serial.println("+-500 deg/s");
            break;
        case MPU6050_RANGE_1000_DEG:
            Serial.println("+-1000 deg/s");
            break;
        case MPU6050_RANGE_2000_DEG:
            Serial.println("+-2000 deg/s");
            break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  // Sets the filter bandwidth
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ:
            Serial.println("260 Hz");
            break;
        case MPU6050_BAND_184_HZ:
            Serial.println("184 Hz");
            break;
        case MPU6050_BAND_94_HZ:
            Serial.println("94 Hz");
            break;
        case MPU6050_BAND_44_HZ:
            Serial.println("44 Hz");
            break;
        case MPU6050_BAND_21_HZ:
            Serial.println("21 Hz");
            break;
        case MPU6050_BAND_10_HZ:
            Serial.println("10 Hz");
            break;
        case MPU6050_BAND_5_HZ:
            Serial.println("5 Hz");
            break;
    }
    Serial.println(""); // does nun jus gives me a break
    delay(100);
}

void loop(){
    sensors_event_t a = {}; // gets me dem values
    sensors_event_t g = {};
    sensors_event_t temp = {};
    mpu.getEvent(&a, &g, &temp);

    Wire.beginTransmission(0x68);

    if (Wire.endTransmission() != 0) {
        Serial.println("IMU LOST!");        // Stops the running of code if MPU fails mid flight
        // autopilotEnabled = false;
        }
    }

    Serial.print("Acceleration X: "); // Prints dem values
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" C");

    Serial.println(""); //gets me dem values printed every 0.5s
    delay(500);

}

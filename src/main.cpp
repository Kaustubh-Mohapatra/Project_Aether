#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float ax, ay, az;
float gx, gy, gz;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

void ReadMPU()
{
    // Sensor readings
    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
}

void calibrateGyro()
{
    const int samples = 2000;

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;

    Serial.println("Calibrating gyro...");
    Serial.println("KEEP THE AIRCRAFT COMPLETELY STILL");

    delay(2000);

    for (int i = 0; i < samples; i++)
    {
        ReadMPU();
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(2);
    }

    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;

    Serial.println("Gyro calibration complete");

    Serial.print("GX bias: ");
    Serial.println(gyroBiasX, 6);

    Serial.print("GY bias: ");
    Serial.println(gyroBiasY, 6);

    Serial.print("GZ bias: ");
    Serial.println(gyroBiasZ, 6);
}

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);
    calibrateGyro();
}

void loop(){
}

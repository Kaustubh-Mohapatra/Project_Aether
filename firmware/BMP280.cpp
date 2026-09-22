#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);
    Serial.println("IMU test");

    if (!bmp.begin(0x76)) {  // 76 or 77
        Serial.println("Barometer initialization check failed");
        while (1){
            delay(10);
        }
    }
    else {
        Serial.println("Barometer initialization successful");
    }
}

void loop()
{
    float h = bmp.readAltitude(1013.25);
    float p = bmp.readPressure();
    float temp = bmp.readTemperature();

    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.print(" °C");

    Serial.print(", Pressure: ");
    Serial.print(p);
    Serial.print(" hPa");

    Serial.print(", Altitude: ");
    Serial.print(h);
    Serial.println(" m");

    delay(100);
}

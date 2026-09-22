#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);
    Serial.println("IMU test");

    if (!bmp.begin(0x76, &Wire)) {  // 76 or 77
        Serial.println("Barometer initialization check failed");
        while (1){
            delay(10);
        }
    }
    else {
        Serial.println("Barometer initialization successful");
    }
}

void loop (){
    sensors_event_t h = {};
    sensors_event_t p = {};
    sensors_event_t temp = {};
    bmp.getEvent(&h, &p, &temp);

    Wire.beginTransmission(0x76);

    Serial.print("Temperature: "); // Prints dem values
    Serial.print(temp.readTemperature());
    Serial.print(" ºC");
    Serial.print(", Pressure: ");
    Serial.print(p.readPressure());
    Serial.print(" Pa");
    Serial.print(", Altitude: ");
    Serial.print(h.readAltitude(1013.25));   // 1013.25 is standard sea level pressure in hPa
    Serial.println(" m");

}

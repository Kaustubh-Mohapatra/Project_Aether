#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp(2);

void setup() {
    Serial.begin(115200);
    Serial.println("BMP280 Test");

    Serial.println("BMP280 SPI Test");
    SPI.begin(18, 19, 23, BMP_CS);

    if (!bmp.begin(0x76))
    {
        Serial.println("BMP initialization failed!");
        while (1) {
            delay(10);
        }
    }
    else    Serial.println("BMP initialization successful");
}

void loop()
{
    float h = bmp.readAltitude();    // 1013.25 is standard sea level pressure in hPa
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

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS   4

// Hardware SPI on ESP32 default VSPI pins
Adafruit_BMP280 bmp(BMP_CS);
// For Software SPI if needed:
// Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

void setup() {
    Serial.begin(115200);
    Serial.println("BMP280 SPI Test");

    SPI.begin(BMP_SCK, BMP_MISO, BMP_MOSI, BMP_CS);

    if (!bmp.begin()) {
        Serial.println("Barometer initialization check failed");
        while (1) {
            delay(10);
        }
    }
    else {
        Serial.println("Barometer initialization successful");
    }
}

void loop()
{
    float h = bmp.readAltitude(1013.25);    // 1013.25 is standard sea level pressure in hPa
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

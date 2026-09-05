#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    if (Wire.endTransmission() != 0) {
        Serial.println("Magnetometer initialization check failed");
        while (1) {
            delay(10);
        }
    }
    else {
        Serial.println("Magnetometer initialization successful");
    }

    // Configuration Register A
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x00);
    Wire.write(0x70);       // 8-average, 15 Hz, normal measurement
    Wire.endTransmission();

    // Configuration Register B
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x01);
    Wire.write(0x20);       // Gain setting
    Wire.endTransmission();

    // Mode Register
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x02);
    Wire.write(0x00);       // Continuous measurement mode
    Wire.endTransmission();

    Serial.println("HMC5883L configured");
    Serial.println("");
    delay(100);
}

void loop() {

    // HMC data registers:
    // X = 0x03, Y = 0x07, Z = 0x05
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);

    Wire.requestFrom(HMC_ADDR, (uint8_t)6);

    if (Wire.available() == 6) {

        int16_t x = (Wire.read() << 8) | Wire.read();
        int16_t z = (Wire.read() << 8) | Wire.read();
        int16_t y = (Wire.read() << 8) | Wire.read();

        Serial.print("Mag X: ");
        Serial.print(x);

        Serial.print(", Y: ");
        Serial.print(y);

        Serial.print(", Z: ");
        Serial.println(z);
    }
    else {
        Serial.println("MAGNETOMETER READ FAILED!");
    }

    delay(500);
}

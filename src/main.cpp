#include <Arduino.h>
#include <Wire.h>

// Generlized the Sensor scanner
void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(21, 22);

    Serial.println("=== I2C SCANNER ===");

    int devices = 0;

    for (uint8_t address = 1; address < 127; address++) {

        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Device found at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);

            devices++;
        }
    }

    Serial.println();

    if (devices == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.print("Found ");
        Serial.print(devices);
        Serial.println(" device(s).");
    }
}

void loop() {}

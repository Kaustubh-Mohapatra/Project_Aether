#include <Wire.h>

#define MPU_ADDR 0x68

uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, (uint8_t)1);

    if (Wire.available()) {
        return Wire.read();
    }

    return 0xFF;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(21, 22);

    Serial.println("=== MPU REGISTER TEST ===");

    Serial.print("WHO_AM_I: 0x");
    Serial.println(readRegister(0x75), HEX);

    Serial.print("PWR_MGMT_1: 0x");
    Serial.println(readRegister(0x6B), HEX);

    Serial.print("GYRO_CONFIG: 0x");
    Serial.println(readRegister(0x1B), HEX);

    Serial.print("ACCEL_CONFIG: 0x");
    Serial.println(readRegister(0x1C), HEX);

    Serial.print("CONFIG: 0x");
    Serial.println(readRegister(0x1A), HEX);

    Serial.print("SMPLRT_DIV: 0x");
    Serial.println(readRegister(0x19), HEX);
}

void loop() {}
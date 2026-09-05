#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_U mag(1);

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    if (!mag.begin(0x1E, &Wire)) {
        Serial.println("Magnetometer initialization failed");
        while (1) {
            delay(10);
        }
    }
    else{
        Serial.println("Magnetometer initialization successful");
    }
}

void loop() {
    sensors_event_t m;
    mag.getEvent(&m);
    mx = m.magnetic.x;
    my = m.magnetic.y;
    mz = m.magnetic.z;

    Serial.print("Mag X: ");
    Serial.print(mx);

    Serial.print(" | Mag Y: ");
    Serial.print(my);

    Serial.print(" | Mag Z: ");
    Serial.println(mz);

    delay(500);
}

#include <Arduino.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

void setup(){
    Serial.begin(115200);
    Wire.begin(21, 22);

    println("Magmeter Test")
    if (!mpu.begin(0x1C, &Wire)) {
        Serial.println("Magmeter initialization check failed");
        while (1){                                  // Stops the running of code if MPU doesnt get detected so we dont end up messing our other stuff up
            delay(10);
        }
    }
    else {
        Serial.println("Magmeter initialization successful");
    }
}

void loop(){

}

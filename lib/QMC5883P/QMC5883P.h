#ifndef QMC5883P_H
#define QMC5883P_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

class QMC5883P
{
public:

    QMC5883P(int32_t sensorID = -1);

    bool begin(TwoWire *wire = &Wire);
    bool read();

    float x();
    float y();
    float z();

    bool getEvent(sensors_event_t *event);

private:

    TwoWire *_wire;
    int32_t _sensorID;
    uint8_t _address;

    float _x;
    float _y;
    float _z;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    bool testAddress(uint8_t address);
};

#endif

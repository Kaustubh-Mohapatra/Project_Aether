#include "QMC5883P.h"

#define QMC5883P_ADDR_PRIMARY 0x2C
#define QMC5883P_ADDR_SECONDARY 0x1C

#define QMC5883P_CHIP_ID_REG 0x00
#define QMC5883P_CHIP_ID     0x80

#define QMC5883P_DATA_REG    0x01
#define QMC5883P_CTRL1_REG   0x0A
#define QMC5883P_CTRL2_REG   0x0B
#define QMC5883P_STATUS_REG  0x09

#define QMC5883P_MODE_CONTINUOUS 0x03

#define QMC5883P_ODR_100HZ       (0x02 << 2)
#define QMC5883P_RANGE_8G        (0x01 << 4)
#define QMC5883P_OSR1_8          (0x00 << 6)
#define QMC5883P_OSR2_1          0x01

#define QMC5883P_CTRL1_VALUE \
    (QMC5883P_OSR1_8 | QMC5883P_RANGE_8G | QMC5883P_ODR_100HZ | QMC5883P_MODE_CONTINUOUS)
#define QMC5883P_CTRL2_VALUE \
    QMC5883P_OSR2_1

QMC5883P::QMC5883P(int32_t sensorID)
{
    _wire = &Wire;
    _sensorID = sensorID;

    _x = 0.0f;
    _y = 0.0f;
    _z = 0.0f;
}

bool QMC5883P::testAddress(uint8_t address)
{
    _wire->beginTransmission(address);
    return _wire->endTransmission() == 0;
}

bool QMC5883P::begin(TwoWire *wire)
{
    _wire = wire;

    if (testAddress(QMC5883P_ADDR_PRIMARY)) {
        _address = QMC5883P_ADDR_PRIMARY;
    }
    else if (testAddress(QMC5883P_ADDR_SECONDARY)) {
        _address = QMC5883P_ADDR_SECONDARY;
    }
    else {
        return false;
    }

    _wire->beginTransmission(_address);

    if (_wire->endTransmission() != 0)
    {
        return false;
    }

    uint8_t chipID;

    if (!readRegisters(QMC5883P_CHIP_ID_REG, &chipID, 1))
        return false;

    if (chipID != QMC5883P_CHIP_ID)
        return false;

    if (!writeRegister(QMC5883P_CTRL2_REG, QMC5883P_CTRL2_VALUE))
        return false;

    if (!writeRegister(QMC5883P_CTRL1_REG, QMC5883P_CTRL1_VALUE))
        return false;

    return true;
}

bool QMC5883P::writeRegister(uint8_t reg, uint8_t value)
{
    _wire->beginTransmission(_address);

    _wire->write(reg);
    _wire->write(value);

    return _wire->endTransmission() == 0;
}

bool QMC5883P::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    _wire->beginTransmission(_address);
    _wire->write(reg);

    if (_wire->endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t received = _wire->requestFrom((uint8_t)_address, length);

    if (received != length)
    {
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = _wire->read();
    }

    return true;
}

bool QMC5883P::read()
{
    uint8_t buffer[6];

    if (!readRegisters(QMC5883P_DATA_REG, buffer, 6))
    {
        return false;
    }

    int16_t rawX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawZ = (int16_t)((buffer[5] << 8) | buffer[4]);

    const float QMC5883P_UT_PER_LSB = 100.0f / 3750.0f;

    _x = rawX * QMC5883P_UT_PER_LSB;
    _y = rawY * QMC5883P_UT_PER_LSB;
    _z = rawZ * QMC5883P_UT_PER_LSB;

    return true;
}

float QMC5883P::x()
{
    return _x;
}

float QMC5883P::y()
{
    return _y;
}

float QMC5883P::z()
{
    return _z;
}

bool QMC5883P::getEvent(sensors_event_t *event)
{
    if (!read())
        return false;

    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    event->timestamp = millis();

    event->magnetic.x = _x;
    event->magnetic.y = _y;
    event->magnetic.z = _z;

    return true;
}

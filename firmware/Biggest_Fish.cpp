#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include "QMC5883P.h"
#include <SD.h>

#define SD_CS 5
#define BMP_CS 4

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp(BMP_CS);
QMC5883P mag(1);
File logFile;
Servo aileron, elevator, rudder;

// Shared snapshot: Core 1 publishes flight data, Core 0 logs it.
struct FlightData
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
    float altitude, pressure;
    int servoAngleP, servoAngleR, servoAngleY;
};

FlightData flightData = {};
portMUX_TYPE flightDataMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sdTaskHandle = NULL;

#define MPU_INT_PIN 27      // drdy bit
volatile bool imuDataReady = false;
void IRAM_ATTR mpuISR(){
    imuDataReady = true;
}

bool ReadMPU();
bool ReadMag();

// Vars

// Quaternion representing orientation
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;


// Filter tuning [Haven't tuned it yet, will tune it later]
float fKp = 3.0f;
float fKi = 0.05f;

// Integral error
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;

// TIMING
unsigned long lastTime = 0;

// IMU
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float declinationAngle = -0.61f * DEG_TO_RAD;
float altitude, pressure;

// IMU Bias correction
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Mag Calibration
// Hard iron offsets
float magBiasX = 0.0f;
float magBiasY = 0.0f;
float magBiasZ = 0.0f;

// Soft iron scales
float magScaleX = 1.0f;
float magScaleY = 1.0f;
float magScaleZ = 1.0f;

void calibrateGyro()
{
    const int samples = 2000;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("Calibrating gyro...");
    Serial.println("Be still for a couble secs");

    delay(2000);

    imuDataReady = false;
    int collected = 0;

    while (collected < samples) {
        if (imuDataReady) {
            imuDataReady = false;

            if (ReadMPU()) {
                sumX += gx;
                sumY += gy;
                sumZ += gz;
                collected++;
            }
        }

        vTaskDelay(1);
    }

    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;

    Serial.println("Calibration complete");

    Serial.print("GX bias: ");
    Serial.println(gyroBiasX, 6);

    Serial.print("GY bias: ");
    Serial.println(gyroBiasY, 6);

    Serial.print("GZ bias: ");
    Serial.println(gyroBiasZ, 6);
}

void calibrateMag()
{
    float minX =  9999.0f;
    float minY =  9999.0f;
    float minZ =  9999.0f;

    float maxX = -9999.0f;
    float maxY = -9999.0f;
    float maxZ = -9999.0f;

    Serial.println("Magnetometer calibration...");
    Serial.println("Rotate the sensor slowly through ALL orientations.");

    delay(2000);

    unsigned long startTime = millis();

    while (millis() - startTime < 15000)
    {
        if (ReadMag())
        {
            minX = min(minX, mx);
            minY = min(minY, my);
            minZ = min(minZ, mz);

            maxX = max(maxX, mx);
            maxY = max(maxY, my);
            maxZ = max(maxZ, mz);
        }

        delay(10);
    }

    magBiasX = (maxX + minX) * 0.5f;
    magBiasY = (maxY + minY) * 0.5f;
    magBiasZ = (maxZ + minZ) * 0.5f;

    // Half-range of each axis
    float radiusX = (maxX - minX) * 0.5f;
    float radiusY = (maxY - minY) * 0.5f;
    float radiusZ = (maxZ - minZ) * 0.5f;

    // Protect against invalid calibration data
    if (radiusX <= 0.0f || radiusY <= 0.0f || radiusZ <= 0.0f)
    {
        Serial.println("Mag calibration failed: invalid axis range");
        magScaleX = 1.0f;
        magScaleY = 1.0f;
        magScaleZ = 1.0f;
        return;
    }

    // Desired common radius
    float averageRadius = (radiusX + radiusY + radiusZ) / 3.0f;

    // Soft-iron scale correction
    magScaleX = averageRadius / radiusX;
    magScaleY = averageRadius / radiusY;
    magScaleZ = averageRadius / radiusZ;

    Serial.println("Mag calibration complete");

    Serial.print("Mag X bias: ");
    Serial.println(magBiasX);

    Serial.print("Mag Y bias: ");
    Serial.println(magBiasY);

    Serial.print("Mag Z bias: ");
    Serial.println(magBiasZ);

    Serial.print("Mag X scale: ");
    Serial.println(magScaleX, 6);

    Serial.print("Mag Y scale: ");
    Serial.println(magScaleY, 6);

    Serial.print("Mag Z scale: ");
    Serial.println(magScaleZ, 6);
}

// MAHONY FILTER

// INV SQRT
float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// MAHONY UPDATE
void MahonyUpdate(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float mx,
    float my,
    float mz,
    float dt
    )
{
    float vx, vy, vz;
    float ex, ey, ez;
    float hx, hy;
    float bx, bz;
    float wx, wy, wz;

    // Normalize accelerometer (fast invSqrt with zero protection)
    float aNormSq = ax * ax + ay * ay + az * az;
    if (aNormSq <= 0.0f)
    {
        return;
    }
    float recipNormA = invSqrt(aNormSq);
    ax *= recipNormA;
    ay *= recipNormA;
    az *= recipNormA;

    // Normalize magnetometer (fast invSqrt with zero protection)
    float mNormSq = mx * mx + my * my + mz * mz;
    if (mNormSq <= 0.0f)
    {
        return;
    }
    float recipNormM = invSqrt(mNormSq);
    mx *= recipNormM;
    my *= recipNormM;
    mz *= recipNormM;

    // Precalculate repeated quaternion products
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Estimated gravity direction from quaternion
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // Estimated magnetic field direction
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3)
       + 2.0f * my * (q1q2 - q0q3)
       + 2.0f * mz * (q1q3 + q0q2);

    hy = 2.0f * mx * (q1q2 + q0q3)
       + 2.0f * my * (0.5f - q1q1 - q3q3)
       + 2.0f * mz * (q2q3 - q0q1);

    bx = sqrtf(hx * hx + hy * hy);

    bz = 2.0f * mx * (q1q3 - q0q2)
       + 2.0f * my * (q2q3 + q0q1)
       + 2.0f * mz * (0.5f - q1q1 - q2q2);

    // Estimated magnetic field from quaternion
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3)
       + 2.0f * bz * (q1q3 - q0q2);

    wy = 2.0f * bx * (q1q2 - q0q3)
       + 2.0f * bz * (q0q1 + q2q3);

    wz = 2.0f * bx * (q0q2 + q1q3)
       + 2.0f * bz * (0.5f - q1q1 - q2q2);

    // Net error
    ex = (ay * vz - az * vy)
       + (my * wz - mz * wy);

    ey = (az * vx - ax * vz)
       + (mz * wx - mx * wz);

    ez = (ax * vy - ay * vx)
       + (mx * wy - my * wx);

    // Integral feedback [In an if so I can turn it off if fKi or fKp = 0]
    if (fKi <= 0.0f)
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    else
    {
        integralFBx += fKi * ex * dt;
        integralFBy += fKi * ey * dt;
        integralFBz += fKi * ez * dt;

        const float iLimit = 0.5f;   // rad/s
        integralFBx = constrain(integralFBx, -iLimit, iLimit);
        integralFBy = constrain(integralFBy, -iLimit, iLimit);
        integralFBz = constrain(integralFBz, -iLimit, iLimit);

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // Proportional feedback
    if (fKp > 0.0f)
    {
        gx += fKp * ex;
        gy += fKp * ey;
        gz += fKp * ez;
    }

    // Quaternion derivative
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    q0 += (-q1 * gx - q2 * gy - q3 * gz);
    q1 += ( q0 * gx + q2 * gz - q3 * gy);
    q2 += ( q0 * gy - q1 * gz + q3 * gx);
    q3 += ( q0 * gz + q1 * gy - q2 * gx);

    // Normalize quaternion
    float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (qNormSq > 0.0f)
    {
        float recipQ = invSqrt(qNormSq);
        q0 *= recipQ;
        q1 *= recipQ;
        q2 *= recipQ;
        q3 *= recipQ;
    }
}

// QUATERNION -> EULER ANGLES
void QuaternionToEuler(
    float &roll,
    float &pitch,
    float &yaw
    )
{
    // Roll
    roll = atan2f(
        2.0f * (q0 * q1 + q2 * q3),
        1.0f - 2.0f * (q1 * q1 + q2 * q2)
    );

    // Pitch
    float sinp = 2.0f * (q0 * q2 - q3 * q1);

    // Prevent asin numerical errors
    if (fabsf(sinp) >= 1.0f)    pitch = copysignf(PI / 2.0f, sinp);
    else    pitch = asinf(sinp);

    // Yaw
    yaw = atan2f(
        2.0f * (q0 * q3 + q1 * q2),
        1.0f - 2.0f * (q2 * q2 + q3 * q3)
    );
    yaw += declinationAngle;

    // Wrap yaw between -PI and +PI
    if (yaw > PI) yaw -= 2.0f * PI;
    else if (yaw < -PI) yaw += 2.0f * PI;

    // Convert radians → degrees
    roll  *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw   *= RAD_TO_DEG;
}

// IMU readings
bool ReadMPU()
{
    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    if (!mpu.getEvent(&a, &g, &temp))   return false;

    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x - gyroBiasX;
    gy = g.gyro.y - gyroBiasY;
    gz = g.gyro.z - gyroBiasZ;

    return true;
}

// Magnetometer readings
bool ReadMag()
{
    sensors_event_t m = {};

    if (!mag.getEvent(&m)) return false;

    mx = (m.magnetic.x - magBiasX) * magScaleX;
    my = (m.magnetic.y - magBiasY) * magScaleY;
    mz = (m.magnetic.z - magBiasZ) * magScaleZ;

    return true;
}

// Barometer readings
bool ReadBMP()
{
    pressure = bmp.readPressure();          // Pa
    altitude = bmp.readAltitude(1013.25f);  // metres, approximate

    return isfinite(pressure) && pressure > 0.0f && isfinite(altitude);
}

// Core 0
void SDTask(void *parameter)
{
    uint32_t lastFlush = millis();

    while (true)
    {
        FlightData snapshot;

        // Snapshot the flight data quickly; never hold the lock during SD I/O.
        portENTER_CRITICAL(&flightDataMux);
        snapshot = flightData;
        portEXIT_CRITICAL(&flightDataMux);

        if (logFile)
        {
            char line[256];

            snprintf(
                line,
                sizeof(line),
                "%lu,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,"
                "%d,%d,%d",

                millis(),

                snapshot.ax, snapshot.ay, snapshot.az,
                snapshot.gx, snapshot.gy, snapshot.gz,
                snapshot.mx, snapshot.my, snapshot.mz,
                snapshot.pitch, snapshot.roll, snapshot.yaw,
                snapshot.altitude,snapshot.pressure,
                snapshot.servoAngleP,
                snapshot.servoAngleR,
                snapshot.servoAngleY
            );

            logFile.println(line);
        }

        // Flush periodically without blocfKing Core 1.
        if (millis() - lastFlush >= 1000)
        {
            lastFlush = millis();

            if (logFile)
                logFile.flush();
        }

        // 100 Hz logging.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);
    Wire.setTimeOut(50);
    aileron.attach(25);
    elevator.attach(32);
    rudder.attach(33);
    SPI.begin(18, 19, 23, SD_CS);
    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    if (!mag.begin())
    {
        Serial.println("Magnetometer initialization failed");
        while (1) {
            delay(10);
          }
    }
    else    Serial.println("Magnetometer initialization successful");


    if (!mpu.begin(0x68, &Wire))
    {
        Serial.println("IMU initialization check failed");
        while (1){
            delay(10);
        }
    }
    else    Serial.println("IMU initialization successful");

    if (!SD.begin(SD_CS, SPI, 4000000))
    {
        Serial.println("SD initialization failed!");
        while(1){
            delay(10);
        }
    }
    else    Serial.println("SD initialization successful");

    if (!bmp.begin(0x76))
    {
        Serial.println("BMP initialization failed!");
        while (1) {
            delay(10);
        }
    }
    else    Serial.println("BMP initialization successful");

    // Shift logging to core 0
    xTaskCreatePinnedToCore(
        SDTask,
        "SD Logger",
        8192,
        NULL,
        1,
        &sdTaskHandle,
        0
    );

    // Configure MPU
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

    // Reading data from MPU with drdy
    // internal sample rate = 1 kHz
    Wire.beginTransmission(0x68);
    Wire.write(0x19);
    Wire.write(9);
    Wire.endTransmission();

    // Enable DATA_RDY interrupt
    Wire.beginTransmission(0x68);
    Wire.write(0x38);      // INT_ENABLE
    Wire.write(0x01);      // DATA_RDY_EN
    Wire.endTransmission();

    // net sample rate = 1khz / (9+1) = 100hz
    // MPU6050 INT
    pinMode(MPU_INT_PIN, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(MPU_INT_PIN),
        mpuISR,
        RISING
    );

    // WE MUST be stationary here
    calibrateGyro();

    // Gotta rotate now
    calibrateMag();

    //Writing onto SD
    char path[24];
    int n = 0;
    do {
        snprintf(path, sizeof(path), "/flight_%04d.csv", n++);
    } while (SD.exists(path) && n < 10000);

    logFile = SD.open(path, FILE_WRITE);
    Serial.print("Logging to "); Serial.println(path);

    if (!logFile)   Serial.println("Failed to open flight.csv!");
    else
    {
        logFile.println(
            "time,"
            "ax,ay,az,"
            "gx,gy,gz,"
            "mx,my,mz,"
            "pitch,roll,yaw,"
            "servoAngleP,servoAngleR,servoAngleY"
        );

        logFile.flush();

        Serial.println("Logging started.");
    }

    // Start timing AFTER calibration
    lastTime = micros();
    Serial.println("Mahony Quaternion Filter Started");
}

void loop()
{
    // Wait for fresh IMU data from interrupt before running loop
    if (!imuDataReady)  return;
    imuDataReady = false;

    // Calculate delta time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    if (dt <= 0.0f || dt > 0.1f)    return;     // filter bad dt

    bool imuOK = ReadMPU();
    bool magOK = ReadMag();
    bool bmpOK = ReadBMP();

    // Run filter
    if (magOK && imuOK) MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // Get Euler angles
    float roll;
    float pitch;
    float yaw;
    float PitchRate = gy * RAD_TO_DEG;
    float RollRate = gx * RAD_TO_DEG;
    float YawRate = gz * RAD_TO_DEG;

    QuaternionToEuler(roll, pitch, yaw);

    // Servo constraints (dependent on the axis of IMU)
    int servoAngleP = constrain(pitch, 20, 160);
    int servoAngleR = constrain(roll, 20, 160);
    int servoAngleY = constrain(yaw, 20, 160);

    // Publish one consistent snapshot for the Core 0 SD logger.
    portENTER_CRITICAL(&flightDataMux);
    flightData.ax = ax;
    flightData.ay = ay;
    flightData.az = az;

    flightData.gx = gx;
    flightData.gy = gy;
    flightData.gz = gz;

    flightData.mx = mx;
    flightData.my = my;
    flightData.mz = mz;

    flightData.pitch = pitch;
    flightData.roll = roll;
    flightData.yaw = yaw;

    flightData.pitchRate = PitchRate;
    flightData.rollRate = RollRate;
    flightData.yawRate = YawRate;

    flightData.altitude = altitude;
    flightData.pressure = pressure;

    flightData.servoAngleP = servoAngleP;
    flightData.servoAngleR = servoAngleR;
    flightData.servoAngleY = servoAngleY;
    portEXIT_CRITICAL(&flightDataMux);

    // Throttle Serial print to ~10 Hz so it does not block the 100 Hz flight loop
    lastPrint = millis();
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.print(roll);
    Serial.print(" | Yaw: ");
    Serial.print(yaw);

    Serial.print(" | Pitch Rate: ");
    Serial.print(PitchRate);
    Serial.print(" | Roll Rate: ");
    Serial.print(RollRate);
    Serial.print(" | Yaw Rate: ");
    Serial.print(YawRate);

    Serial.print(" | Altitude: ");
    Serial.println(altitude);
}

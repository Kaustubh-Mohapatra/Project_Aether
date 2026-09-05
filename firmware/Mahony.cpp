#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_U mag(69);

void ReadMPU();
void ReadMag();

// Vars

// Quaternion representing orientation
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;


// Filter tuning [Haven't tuned it yet, will tune it later]
float Kp = 2.0f;
float Ki = 0.05f;

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

// IMU Bias correction
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Mag Calibration
float magMinX = 0.0f;
float magMinY = 0.0f;
float magMinZ = 0.0f;

float magMaxX = 0.0f;
float magMaxY = 0.0f;
float magMaxZ = 0.0f;

float magBiasX = 0.0f;
float magBiasY = 0.0f;
float magBiasZ = 0.0f;

void calibrateGyro()
{
    const int samples = 2000;

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;

    Serial.println("Calibrating gyro...");
    Serial.println("Be still for a couble secs");

    delay(2000);

    for (int i = 0; i < samples; i++)
    {
        ReadMPU();
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(2);
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
        ReadMag();

        minX = min(minX, mx);
        minY = min(minY, my);
        minZ = min(minZ, mz);

        maxX = max(maxX, mx);
        maxY = max(maxY, my);
        maxZ = max(maxZ, mz);

        delay(10);
    }

    magBiasX = (maxX + minX) * 0.5f;
    magBiasY = (maxY + minY) * 0.5f;
    magBiasZ = (maxZ + minZ) * 0.5f;

    Serial.println("Mag calibration complete");

    Serial.print("Mag X bias: ");
    Serial.println(magBiasX);

    Serial.print("Mag Y bias: ");
    Serial.println(magBiasY);

    Serial.print("Mag Z bias: ");
    Serial.println(magBiasZ);
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
    float normi;
    float normm;
    float vx, vy, vz;
    float ex, ey, ez;
    float hx, hy;
    float bx, bz;
    float wx, wy, wz;

    // Normalize accelerometer
    normi = sqrtf(ax * ax + ay * ay + az * az); // for imu
    normm = sqrtf(mx * mx + my * my + mz * mz); // for mag

    // Prevents division by 0
    if (normi == 0.0f)
    {
        return;
    }

    ax /= normi;
    ay /= normi;
    az /= normi;

    if (normm == 0.0f)
    {
        return;
    }

    mx /= normm;
    my /= normm;
    mz /= normm;

    // Estimated gravity direction from quaternion
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;


    // Estimated magnetic field direction
    hx =
        2.0f * mx * (0.5f - q2*q2 - q3*q3) +
        2.0f * my * (q1*q2 - q0*q3) +
        2.0f * mz * (q1*q3 + q0*q2);

    hy =
        2.0f * mx * (q1*q2 + q0*q3) +
        2.0f * my * (0.5f - q1*q1 - q3*q3) +
        2.0f * mz * (q2*q3 - q0*q1);

    bx = sqrtf(hx*hx + hy*hy);

    bz =
        2.0f * mx * (q1*q3 - q0*q2) +
        2.0f * my * (q2*q3 + q0*q1) +
        2.0f * mz * (0.5f - q1*q1 - q2*q2);


    // Estimated magnetic field from quaternion
    wx =
        2.0f * bx * (0.5f - q2*q2 - q3*q3) +
        2.0f * bz * (q1*q3 - q0*q2);

    wy =
        2.0f * bx * (q1*q2 - q0*q3) +
        2.0f * bz * (q0*q1 + q2*q3);

    wz =
        2.0f * bx * (q0*q2 + q1*q3) +
        2.0f * bz * (0.5f - q1*q1 - q2*q2);


    // Net error
    ex = (ay * vz - az * vy)
       + (my * wz - mz * wy);

    ey = (az * vx - ax * vz)
       + (mz * wx - mx * wz);

    ez = (ax * vy - ay * vx)
       + (mx * wy - my * wx);

    // Integral feedback [In an if so i can turn it off if Ki = 0]
    if (Ki > 0.0f){
        integralFBx += Ki * ex * dt;
        integralFBy += Ki * ey * dt;
        integralFBz += Ki * ez * dt;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // Proportional feedback
    if (Kp > 0.0f){
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    // Quaternion derivative
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    normi = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= normi;
    q1 *= normi;
    q2 *= normi;
    q3 *= normi;
}

// QUATERNION → EULER ANGLES
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
    if (fabsf(sinp) >= 1.0f)
    {
        pitch = copysignf(PI / 2.0f, sinp);
    }
    else
    {
        pitch = asinf(sinp);
    }

    // Yaw
    yaw = atan2f(
        2.0f * (q0 * q3 + q1 * q2),
        1.0f - 2.0f * (q2 * q2 + q3 * q3)
    );

    // Convert radians → degrees
    roll  *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw   *= RAD_TO_DEG;
}

// IMU readings
void ReadMPU()
{
    sensors_event_t a = {};
    sensors_event_t g = {};
    sensors_event_t temp = {};
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x - gyroBiasX;
    gy = g.gyro.y - gyroBiasY;
    gz = g.gyro.z - gyroBiasZ;
}

// Magnetometer readings
void ReadMag()
{
    sensors_event_t m;
    mag.getEvent(&m);
    mx = m.magnetic.x - magBiasX;
    my = m.magnetic.y - magBiasY;
    mz = m.magnetic.z - magBiasZ;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);

    if (!mag.begin()) {
        Serial.println("Magnetometer initialization failed");
        while (1) {
            delay(10);
        }
    }
    else{
        Serial.println("Magnetometer initialization successful");
    }

    Serial.println("IMU test");
    if (!mpu.begin(0x68, &Wire))
    {
        Serial.println("IMU initialization check failed");
        while (1){
            delay(10);
        }
    }
    Serial.println("IMU initialization successful");

    // Configure MPU
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // WE MUST be stationary here
    calibrateGyro();

    // Gotta rotate now
    calibrateMag();

    // Start timing AFTER calibration
    lastTime = micros();
    Serial.println("Mahony Quaternion Filter Started");
}

void loop()
{
    // Calculate delta time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    if (dt <= 0.0f || dt > 0.1f) {                      // Filter bad dt
        return;
    }

    ReadMPU();
    ReadMag();

    // Run filter
    MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // Get Euler angles
    float roll;
    float pitch;
    float yaw;
    float YawRate = gz * RAD_TO_DEG;

    QuaternionToEuler(roll, pitch, yaw);

    Serial.print("Roll: ");
    Serial.print(roll);

    Serial.print(" | Pitch: ");
    Serial.print(pitch);

    Serial.print(" | Yaw Rate: ");
    Serial.print(YawRate);

    Serial.print(" | Mag X: ");
    Serial.print(mx);

    Serial.print(" | Mag Y: ");
    Serial.print(my);

    Serial.print(" | Mag Z: ");
    Serial.println(mz);
    delay(5);
}

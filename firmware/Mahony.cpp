#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void ReadMPU();

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

// MPU
float ax, ay, az;
float gx, gy, gz;

// Bias correction
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

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
    float dt
    )
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // Normalize accelerometer
    norm = sqrtf(ax * ax + ay * ay + az * az);

    // Prevents division by 0
    if (norm == 0.0f)
    {
        return;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated gravity direction from quaternion
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Error between measured and estimated gravity
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

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
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
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

void ReadMPU()
{
    // Sensor readings
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

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);

       Serial.println("MPU6050 test");
       if (!mpu.begin(0x68, &Wire))
       {
            Serial.println("MPU initialization check failed");
            while (1){
                delay(10);
        }
    }

    Serial.println("MPU initialization successful");

    // Configure MPU
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    delay(1000);

    // WE MUST be stationary here
    calibrateGyro();

    // Start timing AFTER calibration
    lastTime = micros();
    Serial.println("Mahony Quaternion Filter Started");
}

void loop()
{
    // Calculate delta time
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.1f) {                      // Filter bad dt
        lastTime = currentTime;
        return;
    }

    ReadMPU();

    // Run filter
    MahonyUpdate(gx, gy, gz, ax, ay, az, dt);

    // Get Euler angles
    float roll;
    float pitch;
    float yaw;

    QuaternionToEuler(roll, pitch, yaw);

    Serial.print("Roll: ");
    Serial.print(roll);

    Serial.print(" | Pitch: ");
    Serial.print(pitch);

    Serial.print(" | Yaw: ");
    Serial.println(yaw);
    delay(5);
}

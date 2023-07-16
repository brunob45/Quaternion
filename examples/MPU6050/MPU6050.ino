#include <Adafruit_MPU6050.h> // https://github.com/adafruit/Adafruit_MPU6050
#include <Quaternion.h>       // https://github.com/carrino/Quaternion
#include <SensorFusion.h>     // https://github.com/aster94/SensorFusion

Adafruit_MPU6050 mpu;
SF fusion;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
}

void loop()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float deltat = fusion.deltatUpdate();
    fusion.MahonyUpdate(
        g.gyro.x, g.gyro.y, g.gyro.z,
        a.acceleration.x, a.acceleration.y, a.acceleration.z,
        deltat);

    // Create the acceleration vector
    Quaternion accel_raw(a.acceleration.x, a.acceleration.y, a.acceleration.z);

    // Get orientation from SensorFusion
    Quaternion attitude;
    memcpy(&attitude.a, fusion.getQuat(), sizeof(float) * 4);

    // Align the acceleration with the NED coordinate system
    Quaternion accel_corrected = attitude.rotate(accel_raw);

    // Print the result
    Serial.print("X: ");
    Serial.println(accel_corrected.x);
    Serial.print("Y: ");
    Serial.println(accel_corrected.y);
    Serial.print("Z: ");
    Serial.println(accel_corrected.z);

    Serial.println();
    delay(500);
}
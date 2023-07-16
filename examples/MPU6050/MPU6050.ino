#include <Adafruit_MPU6050.h> // https://github.com/adafruit/Adafruit_MPU6050
#include <Quaternion.h>       // https://github.com/carrino/Quaternion
#include <SensorFusion.h>     // https://github.com/aster94/SensorFusion

Adafruit_MPU6050 mpu;
SF fusion;
uint32_t last_print;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        yield();
    }

    // Initialize the MPU6050
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            yield();
        }
    }
}

void loop()
{
    // Update sensor data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Update the orientation
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
    // Since we don't have a magnetometer, North is were the board was pointing at boot
    Quaternion accel_ned = attitude.rotate(accel_raw);

    // Print the result every 500ms
    if (millis() - last_print >= 500)
    {
        last_print = millis();
        Serial.print("X: ");
        Serial.println(accel_ned.x);
        Serial.print("Y: ");
        Serial.println(accel_ned.y);
        Serial.print("Z: ");
        Serial.println(accel_ned.z);
        Serial.println();
    }
    delay(50);
}

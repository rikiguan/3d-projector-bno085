#include "Arduino.h"
#include <Wire.h>

#include "esp_log.h"
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
static const char *TAG = "getEular";
extern "C" void setup1(void)
{
    initArduino();
    // Serial.begin(115200);
    // Serial.println();
    // Serial.println("BNO080 Read Example");
    // Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
    delay(100); //  Wait for BNO to boot
    // Start i2c and BNO080
    Wire.flush(); // Reset I2C
    myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
    Wire.begin(48, 47);

    if (myIMU.begin() == false)
    {
        ESP_LOGE(TAG,"BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        
        while (1)
            ;
    }

    Wire.setClock(400000); // Increase I2C data rate to 400kHz

    myIMU.enableRotationVector(50); // Send data update every 50ms

    // Serial.println(F("Rotation vector enabled"));
    // Serial.println(F("Output in form roll, pitch, yaw"));
}

extern "C" int loop1(float* fac_yaw,float* fac_pitch,float* fac_roll)
{
    // Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
        *fac_roll = -(myIMU.getRoll());   // Convert roll to degrees俯仰
        *fac_yaw = -(myIMU.getPitch()); // Convert pitch to degrees横滚
        *fac_pitch = (myIMU.getYaw())-1.63;     // Convert yaw / heading to degrees偏航

        // Serial.print(fac_roll, 1);
        // Serial.print(F(","));
        // Serial.print(fac_pitch, 1);
        // Serial.print(F(","));
        // Serial.print(fac_yaw, 1);

        // Serial.println();

        ESP_LOGI(TAG,"%f,%f,%f",*fac_roll,*fac_pitch,*fac_yaw);
        return 1;
    }
    return 0;
}
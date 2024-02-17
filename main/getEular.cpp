#include "Arduino.h"
#include <Wire.h>
#include "esp_log.h"
#include "freertos/task.h"
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
static const char *TAG = "getEular";




extern "C" void setupBNO085(void)
{
    initArduino();
    delay(100); //  Wait for BNO to boot
    Wire.flush(); // Reset I2C
    myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
    Wire.begin(48, 47);

    if (myIMU.begin() == false)
    {
        ESP_LOGE(TAG,"BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        
        while (1)
            ;
    }
    
    Wire.setClock(2000000); // Increase I2C data rate to 2000kHz
    myIMU.enableRotationVector(1); // Send data update every 50ms

}

extern "C" int getEularData(float* fac_yaw,float* fac_pitch,float* fac_roll)
{
    // Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
        *fac_roll = (myIMU.getRoll());   // Convert roll to degrees俯仰
        *fac_yaw = -(myIMU.getPitch()); // Convert pitch to degrees横滚
        *fac_pitch = -(myIMU.getYaw());     // Convert yaw / heading to degrees偏航
        //ESP_LOGI(TAG,"%f,%f,%f",*fac_roll,*fac_pitch,*fac_yaw);
        return 1;
    }
    return 0;
}
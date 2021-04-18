/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/i2c.h"

#include "MadgwickAHRS.h"

#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

#include "ble.h"
#include "ble_databee.h"
#include "gatt_server.h"

static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

// calibration_t cal = {
//     .mag_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
//     .mag_scale = {.x = 1.0, .y = 1.0, .z = 1.0},
//     .accel_offset = {.x = 0.0, .y = 0.0, .z = 0.0},
//     .accel_scale_lo = {.x = -1.0, .y = -1.0, .z = -1.0},
//     .accel_scale_hi = {.x = 1.0, .y = 1.0, .z = 1.0},
//     .gyro_bias_offset = {.x = 0.0, .y = 0.0, .z = 0.0}
// };

calibration_t cal = {
  .mag_offset = {.x = 2.958984, .y = -7.101562, .z = 58.371094},
  .mag_scale = {.x = 1.046618, .y = 1.033541, .z = 0.952098},
  .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
  .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
  .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
  .gyro_bias_offset = {.x = 2.407645, .y = -1.403642, .z = 0.407150}
};

imu_data_t imu_data;

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void)
{

  i2c_mpu9250_init(&cal);
  MadgwickAHRS_init(SAMPLE_FREQ_Hz, 0.8);

  uint64_t i = 0;
  uint64_t last_time = 0;
  while (true)
  {
    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&imu_data.accel, &imu_data.gyro, &imu_data.magneto));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&imu_data.accel);
    transform_accel_gyro(&imu_data.gyro);
    transform_mag(&imu_data.magneto);

    // Apply the AHRS algorithm
    MadgwickAHRS_update(DEG2RAD(imu_data.gyro.x), DEG2RAD(imu_data.gyro.y), DEG2RAD(imu_data.gyro.z),
                       imu_data.accel.x, imu_data.accel.y, imu_data.accel.z,
                       imu_data.magneto.x, imu_data.magneto.y, imu_data.magneto.z);

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    imu_data.time = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;

    // Print the data out every 10 items
    if (i++ % 10 == 0)
    {
      Madgwick_get_euler_angles_degrees(&imu_data.yaw, &imu_data.pitch, &imu_data.roll);
      ESP_LOGI(TAG, "time diff ms: %f", (imu_data.time - last_time) / 1000.0);
      ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", imu_data.yaw, imu_data.pitch, imu_data.roll);

      // update characteristic value
      update_data_characteristic(imu_data);

      // Make the WDT happy
      esp_task_wdt_reset();
    }

    last_time = imu_data.time;
    mpu_pause();
  }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_RATE_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

void app_main(void)
{
  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ble_init_nimble();
  //start i2c task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}

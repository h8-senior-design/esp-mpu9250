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
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "esp_http_client.h"

#include "driver/i2c.h"

#include "../components/ahrs/MadgwickAHRS.h"
#include "../components/mpu9250/mpu9250.h"
#include "../components/mpu9250/calibrate.h"
#include "../components/mpu9250/common.h"
#include "../components/wifi/station.h"

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

imu_data_t data;

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
  MadgwickAHRSinit(SAMPLE_FREQ_Hz, 0.8);

  uint64_t i = 0;
  while (true)
  {
    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&data.accel, &data.gyro, &data.magneto));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&data.accel);
    transform_accel_gyro(&data.gyro);
    transform_mag(&data.magneto);

    // Apply the AHRS algorithm
    MadgwickAHRSupdate(DEG2RAD(data.gyro.x), DEG2RAD(data.gyro.y), DEG2RAD(data.gyro.z),
                       data.accel.x, data.accel.y, data.accel.z,
                       data.magneto.x, data.magneto.y, data.magneto.z);

    // Print the data out every 10 items
    if (i++ % 10 == 0)
    {

      MadgwickGetEulerAnglesDegrees(&data.yaw, &data.pitch, &data.roll);
      ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°", data.yaw, data.pitch, data.roll);
      ESP_LOGI(TAG, "stack left: %d", uxTaskGetStackHighWaterMark(NULL));

      esp_http_client_config_t config = {
        .url = "http://192.168.0.2:3000/log",
        .method = HTTP_METHOD_POST,
      };
      esp_http_client_handle_t client = esp_http_client_init(&config);
      esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
      esp_http_client_set_post_field(client, (const char *) &data, sizeof(data));
      esp_http_client_perform(client);
      esp_http_client_cleanup(client);

      // Make the WDT happy
      esp_task_wdt_reset();
    }

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
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  wifi_init_sta();
  //start i2c task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}

#ifndef H_BLE_DATABEE_
#define H_BLE_DATABEE_

#include "nimble/ble.h"
#include "host/ble_uuid.h"

#include "mpu9250.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Databee configuration */
static const ble_uuid16_t GATT_DATABEE_UUID = BLE_UUID16_INIT(0x183B);
static const ble_uuid128_t GATT_DATABEE_DATA_UUID = BLE_UUID128_INIT(0xa3, 0x41, 0x9e, 0x5c, 0xf0, 0x3f,
                                                                     0x4e, 0xf1, 0xb6, 0xbf, 0x13, 0x6b, 0x99, 0x67, 0x2e, 0x2c);
#define GATT_DEVICE_INFO_UUID                   0x180A
#define GATT_MANUFACTURER_NAME_UUID             0x2A29
#define GATT_MODEL_NUMBER_UUID                  0x2A24

extern uint16_t databee_data_attr_handle;
extern uint16_t databee_conn_handle;
extern imu_data_t imu_data;
extern TaskHandle_t imu_task_handle;

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

#ifdef __cplusplus
}
#endif

#endif

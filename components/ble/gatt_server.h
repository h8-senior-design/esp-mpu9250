#include "host/ble_gatt.h"

#include "mpu9250.h"

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_server_init(void);
void update_data_characteristic(imu_data_t data);

idf_component_register(SRCS "ble.c" "gatt_server.c"
                       INCLUDE_DIRS "."
                       REQUIRES "bt"
                       PRIV_REQUIRES "mpu9250")

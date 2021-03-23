#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "mpu9250.h"

#include "ble_databee.h"

static const char *TAG = "ble gatt server";
static const char *manuf_name = "ESP32 NimBLE";
static const char *model_num = "NimBLE Databee";

uint16_t databee_data_handle;

static int gatt_svr_chr_access_data(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ble_uuid_cmp(ctxt->chr->uuid, &GATT_DATABEE_DATA_UUID.u)) {
        return os_mbuf_append(ctxt->om, data_buffer, sizeof(imu_data_buffer_t)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_MODEL_NUMBER_UUID) {
        return os_mbuf_append(ctxt->om, model_num, strlen(model_num)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == GATT_MANUFACTURER_NAME_UUID) {
        return os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Data */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DATABEE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Data */
                .uuid = &GATT_DATABEE_DATA_UUID.u,
                .access_cb = gatt_svr_chr_access_data,
                .val_handle = &databee_data_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: * Manufacturer name */
                .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Characteristic: Model number string */
                .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        0, /* No more services */
    },
};

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "registered service %s with handle=%d",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "registering characteristic %s with def_handle=%d val_handle=%d",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "registering descriptor %s with handle=%d",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        break;
    }
}

int gatt_server_init(void) {
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svr_svcs));
  ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svr_svcs));
  return 0;
}

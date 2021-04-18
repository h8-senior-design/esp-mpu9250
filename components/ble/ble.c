#include "gatt_server.h"
#include "ble_databee.h"

#include "esp_log.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

uint16_t databee_conn_handle;
static uint8_t ble_addr_type;
static bool notify_state;

static const char *TAG = "ble";
static const char *device_name = "databee";

static void ble_advertise(void);

void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        ESP_LOGI(TAG, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            ble_advertise();
            break;
        }

        databee_conn_handle = event->connect.conn_handle;
        ble_gattc_exchange_mtu(databee_conn_handle, NULL, NULL);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d\n", event->disconnect.reason);

        ESP_LOGI(TAG, "suspending imu task");
        vTaskSuspend(imu_task_handle);
        /* Connection terminated; resume advertising */
        ble_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "adv complete\n");
        ble_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, databee_data_attr_handle);
        if (event->subscribe.attr_handle == databee_data_attr_handle) {
            notify_state = event->subscribe.cur_notify;
            if (notify_state) {
              ESP_LOGI(TAG, "resuming imu task");
              vTaskResume(imu_task_handle);
            } else {
              ESP_LOGI(TAG, "suspending imu task");
              vTaskSuspend(imu_task_handle);
            }
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", databee_conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;
  }

  return 0;
}

static void ble_on_reset(int reason) {
  ESP_LOGI(TAG, "resetting ble, reason=%d", reason);
}

static void ble_advertise(void) {
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;

  /*
  *  Set the advertisement data included in our advertisements:
  *     o Flags (indicates advertisement type and other general info)
  *     o Advertising tx power
  *     o Device name
  */
  memset(&fields, 0, sizeof(fields));

  /*
  * Advertise two flags:
  *      o Discoverability in forthcoming advertisement (general)
  *      o BLE-only (BR/EDR unsupported)
  */
  fields.flags = BLE_HS_ADV_F_DISC_GEN |
                  BLE_HS_ADV_F_BREDR_UNSUP;

  // Advertise service uuid
  fields.uuids16 = &GATT_DATABEE_UUID;
  fields.num_uuids16 = 1;

  /*
  * Indicate that the TX power level field should be included; have the
  * stack fill this value automatically.  This is done by assigning the
  * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
  */
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  fields.name = (uint8_t *)device_name;
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1;

  ESP_ERROR_CHECK(ble_gap_adv_set_fields(&fields));

  /* Begin advertising */
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  ESP_ERROR_CHECK(ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                                    &adv_params, ble_gap_event, NULL));

}

static void ble_on_sync(void) {
  ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));

  ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &ble_addr_type));
  uint8_t addr[6] = {0};

  ESP_ERROR_CHECK(ble_hs_id_copy_addr(ble_addr_type, addr, NULL));
  ESP_LOGI(TAG, "Device address: %02x:%02x:%02x:%02x:%02x:%02x",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

  // ESP_ERROR_CHECK(ble_att_set_preferred_mtu(150));

  ble_advertise();
}

void ble_init_nimble(void) {
  // initialize nimble host and esp controller
  ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

  nimble_port_init();

  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.reset_cb = ble_on_reset;
  ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

  ESP_ERROR_CHECK(gatt_server_init());

  ESP_ERROR_CHECK(ble_svc_gap_device_name_set(device_name));
  esp_log_level_set("NimBLE", ESP_LOG_NONE);
  // initialize the nimble host configuration
  nimble_port_freertos_init(ble_host_task);
}

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_crc.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"

#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

const char *L_ESPNOW = "ESP_NOW";

uint8_t slaveAddress[] = {0xBC, 0xDD, 0xC2, 0xDB, 0x2A, 0x24};

typedef struct tankData
{
    int tankID;
    int waterLevel;
} tankData;

tankData tankOne;

static void esp_now_receive_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(L_ESPNOW, "Received %d bytes from %s", data_len, macStr);
    memcpy(&tankOne, data, sizeof(tankOne));
    ESP_LOGI(L_ESPNOW, "tankID: %d", tankOne.tankID);
    ESP_LOGI(L_ESPNOW, "waterLevel: %d", tankOne.waterLevel);
}

const esp_now_peer_info_t esp_now_slave = {
    .peer_addr = {0xBC, 0xDD, 0xC2, 0xDB, 0x2A, 0x24},
    .channel = 0,
    .encrypt = false,
    .ifidx = ESP_IF_WIFI_STA};

static void init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&esp_now_slave));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_receive_callback));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init();
    ESP_LOGI(L_ESPNOW, "Master Device");
}

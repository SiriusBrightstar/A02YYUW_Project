#include <stdio.h>

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_crc.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"

#include "driver/gpio.h"

#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

const char *L_ESPNOW = "ESP_NOW";

uint8_t masterAddress[] = {0xA4, 0xE5, 0x7C, 0x74, 0xBE, 0x74};

typedef struct tankData
{
    int tankID;
    int waterLevel;
} tankData;

tankData tankOne;

static void esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(L_ESPNOW, "Message Recieve Status: %s", esp_err_to_name(status));
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        /* GPIO 2's LED will stay ON as long as Messages are recieved successfully */
        gpio_set_level(GPIO_NUM_2, true);
        gpio_set_level(GPIO_NUM_13, false);
    }
    else
    {
        /* GPIO 13's LED will stay ON as long as the reciever fails to get messages */
        gpio_set_level(GPIO_NUM_2, false);
        gpio_set_level(GPIO_NUM_13, true);
    }
}

const esp_now_peer_info_t esp_now_master = {
    .peer_addr = {0xA4, 0xE5, 0x7C, 0x74, 0xBE, 0x74},
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
    ESP_ERROR_CHECK(esp_now_add_peer(&esp_now_master));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_callback));
}

void send_data()
{
    int counter = 0;
    esp_err_t esp_now_send_error;
    while (true)
    {
        tankOne.tankID = counter++;
        tankOne.waterLevel = (counter % 100);
        esp_now_send_error = esp_now_send(masterAddress, (uint8_t *)&tankOne, sizeof(tankOne));
        if (esp_now_send_error == ESP_OK)
        {
            ESP_LOGI(L_ESPNOW, "Data Transmitted Successfully");
        }
        else
        {
            ESP_LOGE(L_ESPNOW, "Error: [%d] %s", esp_now_send_error, esp_err_to_name(esp_now_send_error));
        }
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
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

    /* Initialising LEDs to check if ESP-NOW Messages are successfully sent */
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);

    init();
    ESP_LOGI(L_ESPNOW, "Slave Device");
    xTaskCreate(send_data, "Data Transmitter Task", 1024 * 2, NULL, 2, NULL);
}

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
#include "driver/uart.h"

#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "IOMapping.h"

const char *L_UART = "UART";
const char *L_ESPNOW = "ESP_NOW";
const char *L_DISTANCE_SENSOR = "A02YYUW";

#define Rx_BUFFER 256           // UART Recieve Buffer
#define DATA_LENGTH 4           // Data Length of A02YYUW Sensor
#define TANK_LENGTH 1200        // Tank Length to calculate Water Level
#define DATA_SEND_INTERVAL 5000 // Time Interval between messages in ms

int CauveryTankDistance = 0; // Distance Value in mm

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
        gpio_set_level(LED_D3, true);
    }
    else
    {
        /* GPIO 13's LED will stay ON as long as the reciever fails to get messages */
        gpio_set_level(LED_D3, false);
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
    esp_err_t esp_now_send_error;
    while (true)
    {
        tankOne.tankID = 0;
        tankOne.waterLevel = (TANK_LENGTH - CauveryTankDistance);
        esp_now_send_error = esp_now_send(masterAddress, (uint8_t *)&tankOne, sizeof(tankOne));
        if (esp_now_send_error == ESP_OK)
        {
            ESP_LOGI(L_ESPNOW, "Data Transmitted Successfully");
        }
        else
        {
            ESP_LOGE(L_ESPNOW, "Error: [%d] %s", esp_now_send_error, esp_err_to_name(esp_now_send_error));
        }
        vTaskDelay(DATA_SEND_INTERVAL / portTICK_RATE_MS);
    }
}

/**
 * @brief Get the distance on Cauvery Tank
 *
 */
void getDistance_Cauvery()
{
    uint8_t *data = (uint8_t *)malloc(Rx_BUFFER + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, DATA_LENGTH, (1000 / portTICK_PERIOD_MS));
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGD(L_UART, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(L_UART, data, rxBytes, ESP_LOG_DEBUG);

            int checksum = (data[0] + data[1] + data[2]) & 0x00FF;
            if (checksum == data[3])
            {
                CauveryTankDistance = ((data[1] << 8) + data[2]);
                if ((CauveryTankDistance > 30) || (CauveryTankDistance < 4500))
                {
                    ESP_LOGI(L_DISTANCE_SENSOR, "Distance: %dmm", CauveryTankDistance);
                    ESP_LOGD(L_DISTANCE_SENSOR, "Water Level: %dmm", TANK_LENGTH - CauveryTankDistance);
                }
                else
                {
                    ESP_LOGW(L_DISTANCE_SENSOR, "Sensor Values are out of Range");
                }
            }
            else
            {
                ESP_LOGE(L_DISTANCE_SENSOR, "Checksum Failed");
            }
        }
    }
    free(data);
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

    /* We will set Mode pin to HIGH to get Stable Values (Optional)*/
    gpio_reset_pin(CH4);
    gpio_set_direction(CH4, GPIO_MODE_OUTPUT);
    gpio_set_level(CH4, true);

    /* Initialising LEDs to check if ESP-NOW Messages are successfully sent */
    gpio_reset_pin(LED_D3);
    gpio_set_direction(LED_D3, GPIO_MODE_OUTPUT);

    // gpio_reset_pin(LED_D4);
    // gpio_set_direction(LED_D4, GPIO_MODE_OUTPUT);

    /* Setup UART */
    const uart_port_t cauvery_uart = UART_NUM_2;
    uart_config_t cauvery_uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    };
    ESP_ERROR_CHECK(uart_driver_install(cauvery_uart, Rx_BUFFER, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(cauvery_uart, &cauvery_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, CH5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    init();
    ESP_LOGI(L_ESPNOW, "Slave Device");
    xTaskCreate(getDistance_Cauvery, "Cauvery Tank Sensor", 1024 * 2, NULL, 3, NULL);

    vTaskDelay((DATA_SEND_INTERVAL * 2) / portTICK_RATE_MS);

    xTaskCreate(send_data, "Data Transmitter Task", 1024 * 2, NULL, 2, NULL);
}

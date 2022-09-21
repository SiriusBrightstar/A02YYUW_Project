/**
 * @file        Water_Level_Indicator.c
 * @author      SiriusBrightstar (siriusbrightstar@protonmail.com)
 * @brief       Program for A02YYUW Distance Sensor
 * @version     0.1
 * @date        2022-08-31
 *
 */

#include <stdio.h>

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "IOMapping.h"

/* Logging Tags */

const char *L_UART = "UART";
const char *L_DISTANCE_SENSOR = "A02YYUW";

/* Global Variables */

#define Rx_BUFFER 256       // UART Recieve Buffer
#define DATA_LENGTH 4       // Data Length of A02YYUW Sensor
#define TANK_LENGTH 1200    // Tank Length to calculate Water Level

int CauveryTankDistance = 0; // Distance Value in mm

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
    /* We will set Mode pin to HIGH to get Stable Values (Optional)*/
    gpio_reset_pin(CH4);
    gpio_set_direction(CH4, GPIO_MODE_OUTPUT);
    gpio_set_level(CH4, true);

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

    xTaskCreate(getDistance_Cauvery, "Cauvery Tank Sensor", 1024 * 2, NULL, 3, NULL);
}

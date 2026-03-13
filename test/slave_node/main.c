
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "main";

#define SLAVE_ADDR 0x71
#define SLAVE_SDA GPIO_NUM_21
#define SLAVE_SCL GPIO_NUM_22
#define I2C_PORT I2C_NUM_0

#define RX_BUF_LEN 128
#define TX_BUF_LEN 128

#define MASTER_WRITE_LEN 7
#define MASTER_READ_LEN 5

static const uint8_t slave_tx_data[MASTER_READ_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55};

static void slave_task(void *arg)
{
    uint8_t rx_buf[RX_BUF_LEN];
    i2c_slave_write_buffer(I2C_PORT, (uint8_t *)slave_tx_data, MASTER_READ_LEN, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Slave ready : addr=0x%02X  SDA=%d  SCL=%d", SLAVE_ADDR, (int)SLAVE_SDA, (int)SLAVE_SCL);
    ESP_LOGI(TAG, "TX Buffer Preloaded");

    while (1) {
        int rxd = i2c_slave_read_buffer(I2C_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(50));
        if (rxd <= 0) {
            continue;
        }

        ESP_LOGI(TAG, "Received %d byte(s):", rxd);
        for (int i = 0; i < rxd; i++) {
            ESP_LOGI(TAG, ">>>rx[%d] = 0x%02X", i, rx_buf[i]);
        }

        i2c_slave_write_buffer(I2C_PORT, (uint8_t *)slave_tx_data, MASTER_READ_LEN, pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = SLAVE_SDA,
        .scl_io_num = SLAVE_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = SLAVE_ADDR,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_SLAVE, RX_BUF_LEN, TX_BUF_LEN, 0));

    ESP_LOGI(TAG, "I2C slave driver installed on port %d", I2C_PORT);

    xTaskCreate(slave_task, "i2c_slave", 4096, NULL, 5, NULL);
}


#include "i2c_master.h"

static const char* TAG = "main";

static void test_task (void*);
TaskHandle_t test_task_handle;

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    xTaskCreate(test_task, "Test I2C Driver", 2048, NULL, 3, &test_task_handle);
}

static void test_task (void *pvParameters)
{
    uint8_t read_dat[5], write_dat[7] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x00};
    uint8_t slave_address = 0x71;

    i2c_master_bb_t bus_i2c;
    gpio_num_t sda = 13, scl = 18;
    ESP_ERROR_CHECK(i2c_master_init(&bus_i2c, sda, scl, 400000)); // Fast Mode : 400 KHz
    ESP_LOGI(TAG, "Initialized");
    vTaskDelay(pdMS_TO_TICKS(75));

    esp_err_t err;
    while (1) {        
        err = i2c_master_wr(&bus_i2c, slave_address, write_dat, 7);
        if(err == ESP_OK) {
            ESP_LOGI(TAG, "Transmitted Write Data");
        }
        else {
            ESP_LOGE(TAG, "Transmission Error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(75));

        err = i2c_master_rd(&bus_i2c, slave_address, read_dat, 5);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Received Read Data : %02X %02X %02X %02X %02X", read_dat[0], read_dat[1], read_dat[2], read_dat[3], read_dat[4]);
        }
        else {
            ESP_LOGE(TAG, "Reception Error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(75));

        err = i2c_master_wr_rd(&bus_i2c, slave_address, write_dat, 7, read_dat, 5);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Transmitted Write Data and Received Read Data : %02X %02X %02X %02X %02X", read_dat[0], read_dat[1], read_dat[2], read_dat[3], read_dat[4]);
        }
        else {
            ESP_LOGE(TAG, "Tx/Rx Error: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(75));
    }
}

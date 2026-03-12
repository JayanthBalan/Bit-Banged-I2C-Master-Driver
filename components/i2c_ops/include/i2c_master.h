
#ifndef I2C_MASTER_H
#define I2C_MASTER_H

//Libraries
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_cpu.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Global Macros
#define I2C_MASTER_CLK_STRETCH_TIMEOUT 2000U
#define I2C_MASTER_BUS_CLR_PULSES 9U

//Driver Handle
typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t half_period_cy;
    uint32_t quarter_period_cy;
    portMUX_TYPE spinlock;
    bool initialize;
} i2c_master_bb_t;

/*
return : i2c_master_wr, i2c_master_rd, i2c_master_wr_rd
Slave NACKed = ESP_ERR_INVALID_RESPONSE
Arb lost = ESP_ERR_INVALID_STATE
CLK stretch = ESP_ERR_TIMEOUT
SDA stuck low = ESP_ERR_NOT_FOUND
*/

//API
esp_err_t i2c_master_init(i2c_master_bb_t*, gpio_num_t, gpio_num_t, uint32_t);
esp_err_t i2c_master_deinit(i2c_master_bb_t*);
esp_err_t i2c_master_wr(i2c_master_bb_t*, uint8_t, const uint8_t*, size_t, bool);
esp_err_t i2c_master_rd(i2c_master_bb_t*, uint8_t, uint8_t*, size_t, bool);
esp_err_t i2c_master_wr_rd(i2c_master_bb_t*, uint8_t, const uint8_t*, size_t, uint8_t*, size_t);
bool i2c_master_bus_check(i2c_master_bb_t*);
esp_err_t i2c_master_bus_recover(i2c_master_bb_t*);

#endif

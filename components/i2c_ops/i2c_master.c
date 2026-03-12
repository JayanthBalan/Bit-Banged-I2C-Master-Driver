
#include "i2c_master.h"
 
static const char *TAG = "bb_i2c_master";

// Line Low
#define _PIN_LOW(pin)  do {\
    if ((uint32_t)(pin) < 32U)\
        GPIO.out_w1tc  = (1UL << (uint32_t)(pin));\
    else\
        GPIO.out1_w1tc.data = (1UL << ((uint32_t)(pin) - 32U));\
} while (0)
 
// Line High
#define _PIN_HIGH(pin) do {\
    if ((uint32_t)(pin) < 32U)\
        GPIO.out_w1ts  = (1UL << (uint32_t)(pin));\
    else\
        GPIO.out1_w1ts.data = (1UL << ((uint32_t)(pin) - 32U));\
} while (0)
 
// Line Read
#define _PIN_READ(pin) (((uint32_t)(pin) < 32U) ? ((GPIO.in  >> (uint32_t)(pin)) & 1UL) : ((GPIO.in1.data >> ((uint32_t)(pin) - 32U)) & 1UL))

// Cycle Counter Delay
static IRAM_ATTR inline void _delay_cy (uint32_t cycles)
{
    uint32_t t = esp_cpu_get_cycle_count();
    while ((esp_cpu_get_cycle_count() - t) < cycles) {}
}

// Clock Stretch
static IRAM_ATTR bool _scl_wait(const i2c_master_bb_t *dev)
{
    uint32_t t  = esp_cpu_get_cycle_count();
    uint32_t lim = I2C_MASTER_CLK_STRETCH_TIMEOUT*(uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    while (!_PIN_READ(dev -> scl)) {
        if ((esp_cpu_get_cycle_count() - t) > lim) {
            return false;
        }
    }
    return true;
}

// Bus Primitives
static IRAM_ATTR void _start (const i2c_master_bb_t *dev) // Start Bit
{
    _PIN_HIGH(dev -> sda);
    _PIN_HIGH(dev -> scl);
    _delay_cy((dev) -> half_period_cy);
    _PIN_LOW(dev -> sda);
    _delay_cy((dev) -> half_period_cy);
    _PIN_LOW(dev -> scl);
    _delay_cy((dev) -> quarter_period_cy);
}

static IRAM_ATTR void _repeated_start (const i2c_master_bb_t *dev) // Repeated Start Bit
{
    _PIN_HIGH(dev -> sda);
    _delay_cy((dev) -> quarter_period_cy);
    _PIN_HIGH(dev -> scl);
    _delay_cy((dev) -> half_period_cy);
    _PIN_LOW(dev -> sda);
    _delay_cy((dev) -> half_period_cy);
    _PIN_LOW(dev -> scl);
    _delay_cy((dev) -> quarter_period_cy);
}

static IRAM_ATTR void _stop (const i2c_master_bb_t *dev) // Stop Bit
{
    _PIN_LOW(dev -> sda);
    _delay_cy((dev) -> quarter_period_cy);
    _PIN_HIGH(dev -> scl);
    _delay_cy((dev) -> half_period_cy);
    _PIN_HIGH(dev -> sda);
    _delay_cy((dev) -> half_period_cy);
}

static IRAM_ATTR bool _write_bit (const i2c_master_bb_t *dev, uint8_t bit) // Frame Bit Write
{
    if (bit) {
        _PIN_HIGH(dev -> sda);
    }
    else {
        _PIN_LOW(dev -> sda);
    }
 
    _delay_cy((dev) -> quarter_period_cy);
    _PIN_HIGH(dev -> scl);

    // Clock Stretching Check
    if (!_scl_wait(dev)) {
        return false;
    }

    _delay_cy((dev) -> quarter_period_cy);
    if (bit && !_PIN_READ(dev -> sda)) { // Arbitration
        return false;
    }
 
    _delay_cy((dev) -> quarter_period_cy);
    _PIN_LOW(dev -> scl);
    _delay_cy((dev) -> quarter_period_cy);
 
    return true;
}

static IRAM_ATTR int _read_bit (const i2c_master_bb_t *dev) // Frame Bit Read
{
    _PIN_HIGH(dev -> sda);
    _delay_cy((dev) -> quarter_period_cy);
 
    _PIN_HIGH(dev -> scl);

    // Clock Stretching Check
    if (!_scl_wait(dev)) {
        return -1;
    }

    _delay_cy((dev) -> quarter_period_cy);

    uint8_t bit = (uint8_t)_PIN_READ(dev -> sda);

    _delay_cy((dev) -> quarter_period_cy);
    _PIN_LOW(dev -> scl);
    _delay_cy((dev) -> quarter_period_cy);
 
    return bit;
}

static IRAM_ATTR int8_t _write_byte (const i2c_master_bb_t *dev, uint8_t byte) // Frame Byte Write
{
    for (int8_t i = 7; i >= 0; i--) {
        if (!_write_bit(dev, (byte >> i) & 1u)) {
            return -1;
        }
    }
    return _read_bit(dev);
}

static IRAM_ATTR int16_t _read_byte (const i2c_master_bb_t *dev, bool send_ack) // Frame Byte Read
{
    uint8_t byte = 0;
    int bit;
    for (int8_t i = 7; i >= 0; i--) {
        bit = _read_bit(dev);
        if (bit < 0) {
            return -1;
        }
        byte = (uint8_t)((byte << 1u) | bit);
    }
    _write_bit(dev, (send_ack ? 0u : 1u));
    return byte;
}


// Public API
esp_err_t i2c_master_init (i2c_master_bb_t *dev, gpio_num_t sda, gpio_num_t scl, uint32_t i2c_freq) // Initialize I2C
{
    if (!dev || sda < 0 || scl < 0) { // Constraints
        return ESP_ERR_INVALID_ARG;
    }

    // Memory
    memset(dev, 0, sizeof(*dev));
    dev -> sda = sda;
    dev -> scl = scl;

    // Timing Set
    uint32_t cpu_hz = (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000UL;
    uint32_t full_cy = cpu_hz/i2c_freq;
    dev -> half_period_cy = full_cy/2U;
    dev -> quarter_period_cy = full_cy/4U;

    gpio_config_t io = { //Initialize GPIOs
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        return ret;
    }

    // Initialize I2C Wire
    _PIN_HIGH(dev -> sda);
    _PIN_HIGH(dev -> scl);

    portMUX_INITIALIZE(&dev -> spinlock); // Initialize Spinlock
    dev -> initialize = true; // Initialization Complete
 
    ESP_LOGI(TAG, "Parameters: SDA = %d; SCL = %d; speed = %lu kHz; half = %lu cy; CPU = %lu MHz;", (int)sda, (int)scl, (unsigned long)i2c_freq, (unsigned long)dev -> half_period_cy, (unsigned long)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    return ESP_OK;
}
 
esp_err_t i2c_master_deinit (i2c_master_bb_t *dev) // De-Initialize I2C
{
    if (!dev || !dev -> initialize) { // Constraints
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_direction(dev -> sda, GPIO_MODE_INPUT);
    gpio_set_direction(dev -> scl, GPIO_MODE_INPUT);
    dev -> initialize = false;

    return ESP_OK;
}

esp_err_t i2c_master_wr (i2c_master_bb_t *dev, uint8_t addr, const uint8_t *data, size_t len) // Write Frame
{
    // Constraints
    if (!dev || !dev -> initialize) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > 0 && !data) {
        return ESP_ERR_INVALID_ARG;
    }
 
    esp_err_t ret = ESP_OK;

    // Frame Begin
    vTaskSuspendAll();
    portENTER_CRITICAL(&dev -> spinlock);
 
    _start(dev);

    int8_t ack = _write_byte(dev, (uint8_t)((addr << 1u) | 0u));
    if (ack < 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto no_stop_wr;
    }
    if (ack > 0) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto stop_wr;
    }
 
    for (size_t i = 0; i < len; i++) {
        ack = _write_byte(dev, data[i]);
        if (ack < 0) {
            ret = ESP_ERR_INVALID_STATE;
            goto no_stop_wr;
        }
        if (ack > 0) {
            ret = ESP_ERR_INVALID_RESPONSE;
            goto stop_wr;
        }
    }
 
stop_wr:
    _stop(dev);

no_stop_wr:
    portEXIT_CRITICAL(&dev -> spinlock);
    xTaskResumeAll();
    // Frame End

    return ret;
}

esp_err_t i2c_master_rd (i2c_master_bb_t *dev, uint8_t addr, uint8_t *data, size_t len) // Read Frame
{
    // Constraints
    if (!dev || !dev -> initialize) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!len || !data) {
        return ESP_ERR_INVALID_ARG;
    }
 
    esp_err_t ret = ESP_OK;

    // Frame Begin
    vTaskSuspendAll();
    portENTER_CRITICAL(&dev -> spinlock);
 
    _start(dev);


    int8_t ack = _write_byte(dev, (uint8_t)((addr << 1u) | 1u));
    if (ack < 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto no_stop_rd;
    }
    if (ack > 0) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto stop_rd;
    }
 
    for (size_t i = 0; i < len; i++) {
        int16_t byte = _read_byte(dev, i < len - 1U);
        if (byte < 0) {
            ret = ESP_ERR_TIMEOUT;
            goto stop_rd;
        }
        data[i] = (uint8_t)(byte & 0xFF);
    }
 
stop_rd:
    _stop(dev);

no_stop_rd:
    portEXIT_CRITICAL(&dev -> spinlock);
    xTaskResumeAll();
    // Frame End

    return ret;
}
 
esp_err_t i2c_master_wr_rd (i2c_master_bb_t *dev, uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen) // ||Write -> Read|| Frame
{
    // Constraints
    if (!dev || !dev -> initialize) {
        return ESP_ERR_INVALID_ARG;
    }
    if (wlen && !wdata) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!rlen || !rdata) {
        return ESP_ERR_INVALID_ARG;
    }
 
    esp_err_t ret = ESP_OK;
 
    // Frame Begin
    vTaskSuspendAll();
    portENTER_CRITICAL(&dev -> spinlock);

    _start(dev);
 
    int8_t ack = _write_byte(dev, (uint8_t)((addr << 1u) | 0u));
    if (ack < 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto no_stop_rw;
    }
    if (ack > 0) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto stop_rw;
    }
 
    for (size_t i = 0; i < wlen; i++) {
        ack = _write_byte(dev, wdata[i]);
        if (ack < 0) {
            ret = ESP_ERR_INVALID_STATE;
            goto no_stop_rw;
        }
        if (ack > 0) {
            ret = ESP_ERR_INVALID_RESPONSE;
            goto stop_rw;
        }
    }

    _repeated_start(dev);
 
    ack = _write_byte(dev, (uint8_t)((addr << 1u) | 1u));
    if (ack < 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto no_stop_rw;
    }
    if (ack > 0) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto stop_rw;
    }
 
    for (size_t i = 0; i < rlen; i++) {
        int16_t byte = _read_byte(dev, i < rlen - 1U);
        if (byte < 0) {
            ret = ESP_ERR_TIMEOUT;
            goto stop_rw;
        }
        rdata[i] = (uint8_t)(byte & 0xFF);
    }
 
stop_rw:
    _stop(dev);

no_stop_rw:
    portEXIT_CRITICAL(&dev -> spinlock);
    xTaskResumeAll();
    // Frame End

    return ret;
}

// Utilities
bool i2c_master_bus_check (i2c_master_bb_t *dev) // Idle Check
{
    if (!dev || !dev -> initialize) {
        return false;
    }

    return (_PIN_READ(dev -> sda) && _PIN_READ(dev -> scl));
}
 
esp_err_t i2c_master_bus_recover (i2c_master_bb_t *dev) // Bus Takeover
{
    if (!dev || !dev -> initialize) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "Recovery : SDA = %d; SCL = %d;", (int)dev -> sda, (int)dev -> scl);
 
    vTaskSuspendAll();
    portENTER_CRITICAL(&dev -> spinlock);
 
    _PIN_HIGH(dev -> sda);
    _PIN_HIGH(dev -> scl);
    _delay_cy((dev) -> half_period_cy);
 
    for (uint8_t i = 0; i < I2C_MASTER_BUS_CLR_PULSES; i++) {
        if (_PIN_READ(dev -> sda)) {
            break;
        }

        _PIN_LOW(dev -> scl);
        _delay_cy((dev) -> half_period_cy);
        _PIN_HIGH(dev -> scl);
        _delay_cy((dev) -> half_period_cy);
    }

    _stop(dev);
 
    portEXIT_CRITICAL(&dev -> spinlock);
    xTaskResumeAll();
 
    esp_err_t ret = i2c_master_bus_check(dev) ? ESP_OK : ESP_ERR_NOT_FOUND;
    ESP_LOGI(TAG, "Recovery %s", ret == ESP_OK ? "OK" : "FAILED");

    return ret;
}

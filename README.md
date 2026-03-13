# ESP32 Bit-Banged I2C Master Driver

A fully software-driven I2C master implementation for the ESP32 (ESP-WROOM-32), written in Embedded C using the ESP-IDF framework. Rather than using the ESP32's built-in I2C peripheral, this driver manually controls GPIO lines through direct register writes — giving complete visibility and control over every bit on the bus.

This was built from scratch as a learning project, going through multiple iterations of design review, bug fixes, and logic analyzer verification. All three transaction types — write, read, and combined write-then-read — are verified working on real hardware.

---

## Why Bit-Bang?

The ESP32 has two hardware I2C peripherals, so why do this in software? A few reasons:

- Full transparency over bus timing — you can see and control every single clock edge
- Works on any GPIO pair, not just the hardware I2C pins
- Deep understanding of the I2C protocol comes from implementing it yourself
- Useful when the hardware peripheral is already occupied or unavailable
- Easier to debug — if something is wrong, the problem is in your own code

The tradeoff is CPU usage. This driver suspends the FreeRTOS scheduler and disables interrupts for the duration of every frame to guarantee bit-accurate timing. For most embedded applications where I2C transactions are infrequent and short, this is completely acceptable.

---

## Features

- Direct register GPIO access (`GPIO.out_w1ts` / `GPIO.out_w1tc`) — single-cycle APB writes, no driver overhead
- CPU cycle counter timing (`esp_cpu_get_cycle_count`) — sub-microsecond resolution at any clock frequency
- Full task isolation — `vTaskSuspendAll()` + `portENTER_CRITICAL()` for both cores
- Multi-master bus arbitration — detects when another master wins the bus mid-frame
- Clock stretching support — slave can hold SCL low; driver polls and times out cleanly
- Open-drain GPIO mode — hardware OD, no risk of bus contention
- 7-bit addressing
- Three transaction types: write, read, combined write-read (repeated START)
- Bus recovery — clocks up to 9 SCL pulses to free a slave stuck holding SDA low
- Configurable clock frequency — pass any value, works from 100 kHz to 400 kHz comfortably
- Clean error propagation — distinct `esp_err_t` codes for NACK, arbitration loss, clock stretch timeout, and bus stuck

---

## Project Structure

```
TRANSPARENTWORLD/
├── components/
│   └── i2c_ops/
│       ├── include/
│       │   └── i2c_master.h        # Driver handle, macros, API declarations
│       ├── i2c_master.c            # Full driver implementation
│       └── CMakeLists.txt
├── main/
│   ├── main.c                      # Master application — test task
│   └── CMakeLists.txt
├── CMakeLists.txt
└── sdkconfig
```

The driver lives in its own component under `components/i2c_ops/` so it can be cleanly reused across projects without touching `main/`.

---

## Hardware Setup

Tested on two ESP-WROOM-32 modules — one running the bit-bang master, one running the ESP-IDF built-in I2C slave.

```
Master ESP32                        Slave ESP32
─────────────────────────────────────────────────
GPIO 13  (SDA) ──────────────────── GPIO 21 (SDA)
GPIO 18  (SCL) ──────────────────── GPIO 22 (SCL)
GND            ──────────────────── GND

Pull-ups (shared on the bus):
  SDA line ── 2.2 kΩ ── 3.3 V
  SCL line ── 2.2 kΩ ── 3.3 V
```

**Why 2.2 kΩ?** I2C is an open-drain bus — neither master nor slave ever drives the line HIGH. The pull-up resistor is what brings the line high when everyone lets go. The resistor value and bus capacitance form an RC circuit that determines how fast the line rises. At 400 kHz (Fast Mode), the I2C spec requires the rise time to be under 300 ns. With 2.2 kΩ and typical short-wire capacitance of ~50 pF, the rise time is approximately 93 ns — well within spec. Using 4.7 kΩ at 400 kHz starts pushing the rise time close to the limit and can cause unreliable behaviour.

---

## API

### Initialise

```c
esp_err_t i2c_master_init(i2c_master_bb_t *dev,
                           gpio_num_t sda,
                           gpio_num_t scl,
                           uint32_t i2c_freq);
```

Configures both GPIOs as open-drain input/output, computes half and quarter period cycle counts from the CPU frequency, initialises the spinlock, and releases both lines to idle HIGH.

```c
i2c_master_bb_t bus;
ESP_ERROR_CHECK(i2c_master_init(&bus, GPIO_NUM_13, GPIO_NUM_18, 400000));
```

### Write

```c
esp_err_t i2c_master_wr(i2c_master_bb_t *dev,
                         uint8_t addr,
                         const uint8_t *data,
                         size_t len);
```

Generates: `START | (addr<<1)|W | data[0] ... data[len-1] | STOP`

```c
uint8_t buf[] = {0xA1, 0xB2, 0xC3};
esp_err_t err = i2c_master_wr(&bus, 0x71, buf, 3);
```

### Read

```c
esp_err_t i2c_master_rd(i2c_master_bb_t *dev,
                         uint8_t addr,
                         uint8_t *data,
                         size_t len);
```

Generates: `START | (addr<<1)|R | data[0]+ACK ... data[len-1]+NACK | STOP`

The master ACKs every byte except the last, which gets a NACK to signal the slave to stop.

```c
uint8_t result[5];
esp_err_t err = i2c_master_rd(&bus, 0x71, result, 5);
```

### Write then Read (Repeated START)

```c
esp_err_t i2c_master_wr_rd(i2c_master_bb_t *dev,
                             uint8_t addr,
                             const uint8_t *wdata, size_t wlen,
                             uint8_t *rdata,  size_t rlen);
```

Generates: `START | ADDR+W | wdata | Sr | ADDR+R | rdata | STOP`

The bus is never released between the write and read phases. This is the standard way to read a register from a sensor — write the register address, then read back the value without another master being able to steal the bus in between.

```c
uint8_t reg = 0x3B;
uint8_t sensor_data[6];
esp_err_t err = i2c_master_wr_rd(&bus, 0x68, &reg, 1, sensor_data, 6);
```

### Utilities

```c
bool      i2c_master_bus_check(i2c_master_bb_t *dev);   // true if both lines are HIGH (bus idle)
esp_err_t i2c_master_bus_recover(i2c_master_bb_t *dev); // clock 9 pulses to free stuck slave
esp_err_t i2c_master_deinit(i2c_master_bb_t *dev);      // float both GPIOs
```

### Error Codes

| Return Value | Meaning |
|---|---|
| `ESP_OK` | Transaction completed successfully |
| `ESP_ERR_INVALID_RESPONSE` | Slave sent NACK — not present, not ready, or rejected data |
| `ESP_ERR_INVALID_STATE` | Arbitration lost — another master took the bus |
| `ESP_ERR_TIMEOUT` | Slave held SCL low beyond the 2000 µs clock-stretch timeout |
| `ESP_ERR_NOT_FOUND` | Bus recovery failed — SDA stuck low even after 9 clock pulses |
| `ESP_ERR_INVALID_ARG` | NULL pointer or uninitialised device passed to the function |

---

## How the Driver Works Internally

### GPIO toggling

All SDA and SCL transitions use direct writes to the GPIO set/clear registers:

```c
GPIO.out_w1ts = (1UL << pin);   // release HIGH (open-drain: pull-up takes over)
GPIO.out_w1tc = (1UL << pin);   // drive LOW    (active pull-down)
```

These are single-cycle 32-bit APB register writes — roughly 4–5× faster than `gpio_set_level()`, which goes through the driver abstraction layer. At 400 kHz every nanosecond counts, and this keeps the timing clean.

### Bit timing

Each SCL period is divided into four equal quarter-period slots, each measured by polling the Xtensa CCOUNT cycle counter:

```
[1] Drive SDA           (quarter period — data setup time)
[2] Raise SCL           → poll until slave releases it (clock stretch)
[3] Wait                (quarter period — line settling before sample)
[4] Sample SDA          → arbitration check if we sent 1
[5] Wait                (quarter period — data hold time)
[6] Drop SCL
[7] Wait                (quarter period — SCL low time)
```

Step 3 is important: the quarter-period gap between SCL rising and the SDA sample gives the 2.2 kΩ pull-up time to charge the bus capacitance to a valid HIGH before the sample. Without this delay, false arbitration-lost errors would appear at higher speeds.

### Task isolation

```c
vTaskSuspendAll();                      // stop FreeRTOS scheduler on this core
portENTER_CRITICAL(&dev->spinlock);     // cross-core spinlock + disable IRQs
// ... entire I2C frame ...
portEXIT_CRITICAL(&dev->spinlock);
xTaskResumeAll();
```

`vTaskSuspendAll()` prevents context switches on the calling core. `portENTER_CRITICAL()` with a per-instance spinlock handles the second core — any code on Core 1 trying to acquire the same spinlock will spin-wait until the frame is complete. Together these ensure zero scheduler or interrupt jitter during transmission.

### Arbitration

After driving SDA HIGH (releasing the line) during each bit, the driver samples SDA after the SCL rise and settling delay. If SDA reads LOW while we drove HIGH, another master is holding it down and has won arbitration. The frame is abandoned immediately **without generating STOP** — this is required by the I2C spec, because generating STOP would corrupt the winning master's ongoing transaction.

### Clock stretching

After raising SCL, the driver polls SCL until it actually reads HIGH. A slave can hold SCL low after the master raises it to signal "I'm not ready yet." The driver just waits. If the slave holds it low for more than `I2C_MASTER_CLK_STRETCH_TIMEOUT` microseconds (default 2000 µs), `_scl_wait()` returns false and the frame aborts with `ESP_ERR_TIMEOUT`. The timeout is checked via the CCOUNT register so it is accurate even with interrupts disabled.

### Error sentinel in `_read_byte`

`_read_byte()` returns `int16_t` rather than `uint8_t`. This is intentional — all 256 valid byte values (0x00–0xFF) are representable as positive `int16_t` values, and -1 is used as an error sentinel for clock-stretch timeout. Using `uint8_t` would make it impossible to distinguish a genuine 0x00 byte from a timeout. Using `int8_t` would corrupt bytes above 0x7F. At the call site, the check and cast pattern is:

```c
int16_t byte = _read_byte(dev, send_ack);
if (byte < 0) { ret = ESP_ERR_TIMEOUT; goto stop; }
data[i] = (uint8_t)(byte & 0xFF);
```

---

## Verified Waveforms

All three transaction types were captured with a logic analyser and decoded as I2C.

### Master Write — `i2c_master_wr`

Address `0xE2` (wire format) = `0x71` (7-bit) with Write bit. Seven data bytes `A1 B2 C3 D4 E5 F6 00` transmitted, each acknowledged by the slave.

```
S | AW:E2 A | A1 A | B2 A | C3 A | D4 A | E5 A | F6 A | 00 A | P
```

### Master Read — `i2c_master_rd`

Address `0xE3` (wire format) = `0x71` with Read bit. Five bytes `11 22 33 44 55` returned by the slave. Master ACKs the first four and NACKs the fifth to end the transaction.

```
S | AR:E3 A | 11 A | 22 A | 33 A | 44 A | 55 N | P
```

### Combined Write-Read — `i2c_master_wr_rd`

Write phase followed immediately by a repeated START and read phase without releasing the bus.

```
S | AW:E2 A | A1 A | B2 A | C3 A | D4 A | E5 A | F6 A | 00 A | Sr | AR:E3 A | 11 A | 22 A | 33 A | 44 A | 55 N | P
```

### Clock Frequency

Measured SCL period at 400 kHz setting: **3500 ns** (285.7 kHz actual). The ~30% frequency droop from the nominal 400 kHz comes from the overhead of the arbitration check, `_scl_wait()` poll loop iteration, and the GPIO register read in `_PIN_READ()` all consuming cycles within the quarter-period budget. At 160 MHz CPU and 400 kHz target, each quarter period is only 100 cycles — tight enough that this overhead is visible. The resulting ~285 kHz is still well within Fast Mode electrical requirements.

---

## Clock Frequency and Pull-up Guide

| Target Frequency | Actual (approx.) | Recommended Pull-up | Notes |
|---|---|---|---|
| 100 kHz | ~95 kHz | 4.7 kΩ | Standard Mode, very relaxed timing |
| 200 kHz | ~185 kHz | 2.2–4.7 kΩ | Stable, good headroom |
| 400 kHz | ~285 kHz | 2.2 kΩ | Fast Mode ceiling, actual ~285 kHz due to overhead |

Any frequency value can be passed to `i2c_master_init()` — the timing calculation is `cpu_hz / freq` and works for any number. Standard (100 kHz) and Fast (400 kHz) are the only officially named modes in the I2C spec; anything in between is perfectly valid and is just referred to by its frequency.

---

## Building and Flashing

Standard ESP-IDF build process. Requires ESP-IDF v5.x.

```bash
idf.py set-target esp32
idf.py build
idf.py -p PORT flash monitor
```

The driver component is in `components/i2c_ops/`. The root `CMakeLists.txt` picks it up automatically. No additional configuration needed.

---

## Configuration

Constants in `i2c_master.h`:

```c
#define I2C_MASTER_CLK_STRETCH_TIMEOUT  2000U   // max µs slave may hold SCL low
#define I2C_MASTER_BUS_CLR_PULSES       9U       // SCL pulses during bus recovery
```

These can be adjusted without touching the driver logic.

---

## Limitations

- 7-bit addressing only (10-bit not implemented)
- No DMA — every bit is clocked by the CPU
- Scheduler and interrupts are suspended for the full duration of each frame — keep transactions short if real-time response matters elsewhere in the application
- Actual SCL frequency is lower than requested due to software overhead in the bit-loop — at 400 kHz the real frequency measures approximately 285 kHz
- Chaining two `i2c_master_wr` calls with no STOP between them is not atomic — use `i2c_master_wr_rd` for combined transactions

---

## License

MIT — do what you want with it.

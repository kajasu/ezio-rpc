#include "modbus.h"
#include "ezapp.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "modbus";
// UART pins and settings for Modbus RTU slave
#define MODBUS_UART_NUM UART_NUM_1
#define MODBUS_TX_PIN 27
#define MODBUS_RX_PIN 14

#define MODBUS_BAUDRATE 19200
// Slave unit id
#define MODBUS_UNIT_ID 1

// Optional DE (driver enable) pin for RS485 transceiver; set to -1 if not used
#define MODBUS_DE_PIN -1



static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

static int build_modbus_fc03_request(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t *out, int out_size)
{
    if (!out || out_size < 8) return 0;
    out[0] = slave_id;
    out[1] = 0x03;
    out[2] = (start_addr >> 8) & 0xFF;
    out[3] = start_addr & 0xFF;
    out[4] = (count >> 8) & 0xFF;
    out[5] = count & 0xFF;
    uint16_t crc = modbus_crc16(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
    return 8;
}

// FC04 (Read Input Registers) request format is identical to FC03 request
static int build_modbus_fc04_request(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t *out, int out_size)
{
    // same layout as FC03: slave(1) func(1) addr(2) count(2) crc(2)
    return build_modbus_fc03_request(slave_id, start_addr, count, out, out_size);
}

// FC16 / 0x10 (Write Multiple Registers) request builder
// request: [slave][0x10][addr_hi][addr_lo][count_hi][count_lo][bytecount][data...][crc_lo][crc_hi]
static int build_modbus_fc10_request(uint8_t slave_id, uint16_t start_addr, uint16_t count, const uint8_t *data_bytes, uint8_t *out, int out_size)
{
    if (!out || !data_bytes) return 0;
    if (count == 0 || count > 123) return 0; // Modbus limit for registers in a single write
    int bytecount = count * 2;
    int needed = 9 + bytecount; // 1+1+2+2+1 + bytecount + 2 CRC
    if (out_size < needed) return 0;

    out[0] = slave_id;
    out[1] = 0x10;
    out[2] = (start_addr >> 8) & 0xFF;
    out[3] = start_addr & 0xFF;
    out[4] = (count >> 8) & 0xFF;
    out[5] = count & 0xFF;
    out[6] = (uint8_t)bytecount;
    memcpy(&out[7], data_bytes, bytecount);
    uint16_t crc = modbus_crc16(out, 7 + bytecount);
    out[7 + bytecount] = crc & 0xFF;
    out[7 + bytecount + 1] = (crc >> 8) & 0xFF;
    return needed;
}

static bool parse_modbus_fc03_response(const uint8_t *buf, int len, uint8_t expected_slave, uint16_t expected_count)
{
    if (!buf) return false;
    // Response: [slave][func][bytecount][data...][crc_lo][crc_hi]
    if (len < 5) return false;
    if (buf[0] != expected_slave) return false;
    if (buf[1] != 0x03) return false;
    uint8_t bytecount = buf[2];
    if (bytecount != expected_count * 2) return false;
    int expected_len = 3 + bytecount + 2;
    if (len < expected_len) return false;
    uint16_t recv_crc = buf[expected_len - 2] | (buf[expected_len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(buf, expected_len - 2);
    if (recv_crc != calc_crc) return false;
    return true;
}

static int uart_read_exact(uart_port_t port, uint8_t *dst, int want, TickType_t timeout_ticks)
{
    int got = 0;
    TickType_t start = xTaskGetTickCount();
    while (got < want) {
        TickType_t now = xTaskGetTickCount();
        if (now - start >= timeout_ticks) break;
        // use short reads so total timeout is respected (small per-call blocking)
        int r = uart_read_bytes(port, dst + got, want - got, pdMS_TO_TICKS(1));
        if (r > 0) got += r;
    }
    return got;
}

static bool uart_write_bytes_blocking(uart_port_t port, const uint8_t *data, size_t len, TickType_t ticks)
{
    int written = uart_write_bytes(port, (const char *)data, len);
    if (written < 0) return false;
    uart_wait_tx_done(port, ticks);
    return true;
}

static bool parse_modbus_frame(uint8_t *buf, int len, uint8_t *out_slave, uint8_t *out_func, uint16_t *out_addr, uint16_t *out_count, uint8_t **out_data, int *out_data_len)
{
    if (len < 5) return false;
    uint8_t slave = buf[0];
    uint8_t func = buf[1];
    // compute expected length based on function
    int expected = 0;
    if (func == 0x03 || func == 0x04) {
        // request length is 8 (addr hi lo, count hi lo, crc lo hi)
        expected = 8;
    } else if (func == 0x10) {
        if (len < 9) return false; // need byte count field
        uint16_t count = (buf[4] << 8) | buf[5];
        int bytecount = buf[6];
        if (bytecount != count * 2) return false;
        expected = 9 + bytecount; // total
    } else {
        return false;
    }
    if (len < expected) return false;
    // verify CRC
    uint16_t recv_crc = buf[expected-2] | (buf[expected-1] << 8);
    uint16_t calc = modbus_crc16(buf, expected-2);
    if (recv_crc != calc) return false;
    *out_slave = slave; *out_func = func;
    *out_addr = (buf[2] << 8) | buf[3];
    *out_count = (buf[4] << 8) | buf[5];
    if (func == 0x10) {
        *out_data = &buf[7];
        *out_data_len = buf[6];
    } else {
        *out_data = NULL;
        *out_data_len = 0;
    }
    return true;
}

static void modbus_task(void *arg)
{
    (void)arg;
    uart_port_t uart_num = MODBUS_UART_NUM;
    EzApp &app = EzApp::instance();
    const int bufsize = 512;
    uint8_t buf[bufsize];
    int idx = 0;

    while (1) {
        int r = uart_read_bytes(uart_num, buf + idx, bufsize - idx, pdMS_TO_TICKS(200));
        if (r > 0) {
            idx += r;
            // try to parse frames; consume processed bytes
            int processed = 0;
            for (int offset = 0; offset < idx; ) {
                uint8_t slave=0, func=0; uint16_t addr=0, count=0; uint8_t *data=nullptr; int data_len=0;
                int rem = idx - offset;
                if (!parse_modbus_frame(buf + offset, rem, &slave, &func, &addr, &count, &data, &data_len)) {
                    // if not enough data to parse, break and wait for more
                    break;
                }
                // compute frame length
                int frame_len = 0;
                if (func == 0x03 || func == 0x04) frame_len = 8;
                else if (func == 0x10) frame_len = 9 + data_len;

                // only respond to our unit id
                if (slave == MODBUS_UNIT_ID) {
                    if (func == 0x03 || func == 0x04) {
                        // Read registers from EzApp::D, mapping register N -> D offset N*2
                        if (count == 0 || count > 125) {
                            // invalid count; ignore
                        } else {
                            int bytecount = count * 2;
                            uint8_t resp[5 + 250];
                            resp[0] = MODBUS_UNIT_ID;
                            resp[1] = func;
                            resp[2] = bytecount;
                            for (int i = 0; i < count; ++i) {
                                int16_t v = 0;
                                uint32_t off = (uint32_t)addr + i; // register -> D index
                                app.readInt16(EzApp::D, off * 2, v);
                                resp[3 + i*2] = (v >> 8) & 0xFF;
                                resp[3 + i*2 + 1] = v & 0xFF;
                            }
                            uint16_t crc = modbus_crc16(resp, 3 + bytecount);
                            resp[3 + bytecount] = crc & 0xFF;
                            resp[3 + bytecount + 1] = (crc >> 8) & 0xFF;
                            uart_write_bytes_blocking(uart_num, resp, 3 + bytecount + 2, pdMS_TO_TICKS(200));
                        }
                    } else if (func == 0x10) {
                        // Write multiple registers
                        uint16_t count_regs = count;
                        if (count_regs == 0 || count_regs > 123) {
                            // invalid, ignore
                        } else {
                            // data points to bytes; write each register
                            for (int i = 0; i < count_regs; ++i) {
                                uint16_t hi = data[i*2];
                                uint16_t lo = data[i*2 + 1];
                                int16_t val = (int16_t)((hi << 8) | lo);
                                uint32_t off = (uint32_t)addr + i;
                                app.writeInt16(EzApp::D, off * 2, val);
                            }
                            // respond with echo of address and count
                            uint8_t resp[8];
                            resp[0] = MODBUS_UNIT_ID;
                            resp[1] = 0x10;
                            resp[2] = (addr >> 8) & 0xFF;
                            resp[3] = addr & 0xFF;
                            resp[4] = (count_regs >> 8) & 0xFF;
                            resp[5] = count_regs & 0xFF;
                            uint16_t crc = modbus_crc16(resp, 6);
                            resp[6] = crc & 0xFF;
                            resp[7] = (crc >> 8) & 0xFF;
                            uart_write_bytes_blocking(uart_num, resp, 8, pdMS_TO_TICKS(200));
                        }
                    }
                }

                offset += frame_len;
                processed = offset;
            }
            // shift remaining bytes
            if (processed > 0) {
                memmove(buf, buf + processed, idx - processed);
                idx -= processed;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

// configure UART and start task
void start_modbus_task()
{
    // UART init for Modbus RTU slave (TX=GPIO32, RX=GPIO33) at 115200
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = MODBUS_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    uart_param_config(MODBUS_UART_NUM, &uart_config);
    uart_set_pin(MODBUS_UART_NUM, MODBUS_TX_PIN, MODBUS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MODBUS_UART_NUM, 2048, 0, 0, NULL, 0);

    xTaskCreate(modbus_task, "modbus", 8192, NULL, 5, NULL);
}

static void modbus_master_task(void *arg)
{
    (void)arg;
    const uart_port_t uart_num = MODBUS_UART_NUM;

    // Parameters requested by user
    const uint8_t slave_id = 1;
    const uint16_t start_addr = 1;
    const uint16_t count = 2;
    const TickType_t rx_timeout = pdMS_TO_TICKS(20); // increased to 20 ms
    const TickType_t period = pdMS_TO_TICKS(500); // 500 ms

    uint8_t req[8];
    const int req_len = build_modbus_fc03_request(slave_id, start_addr, count, req, sizeof(req));
    if (req_len <= 0) {
        ESP_LOGE(TAG, "Failed to build Modbus FC03 request");
        vTaskDelete(NULL);
        return;
    }

    const int bytecount = count * 2;
    const int resp_len = 3 + bytecount + 2;
    uint8_t resp[32];

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        // flush any stale bytes
        uart_flush_input(uart_num);
            // if DE pin is used, enable driver before transmit
            if (MODBUS_DE_PIN >= 0) {
                gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (!uart_write_bytes_blocking(uart_num, req, req_len, pdMS_TO_TICKS(200))) {
            ESP_LOGW(TAG, "Modbus master TX failed");
        } else {
            int got = uart_read_exact(uart_num, resp, resp_len, rx_timeout);
            if (got == resp_len && parse_modbus_fc03_response(resp, got, slave_id, count)) {
                int16_t v = ((int16_t)resp[3] << 8) | resp[4];
                v *= 10;
                EzApp::instance().writeInt16(EzApp::D, 110, v);
                int16_t v2 = ((int16_t)resp[5] << 8) | resp[6];
                int16_t d140_val = 0;
                EzApp::instance().readInt16(EzApp::D, 140, d140_val);
                //
                ESP_LOGI(TAG, "Modbus master RX success: Reg[%d]=%d,%d", start_addr, v, v2);
                if (v2 != d140_val) {
                    int16_t write_val = d140_val / 10;
                    uint8_t data[4];
                    data[0] = (write_val >> 8) & 0xFF;
                    data[1] = write_val & 0xFF;
                    uint8_t write_req[16];
                    int write_len = build_modbus_fc10_request(slave_id, 300, 1, data, write_req, sizeof(write_req));
                    if (write_len > 0) {
                        uart_write_bytes_blocking(uart_num, write_req, write_len, pdMS_TO_TICKS(200));
                    }
                }
            } else {
                if (got > 0) {
                    ESP_LOGW(TAG, "Modbus master RX invalid/timeout (got=%d need=%d)", got, resp_len);
                } else {
                    ESP_LOGW(TAG, "Modbus master RX timeout");
                }
            }
        }
            // after TX complete, disable driver to listen
            if (MODBUS_DE_PIN >= 0) {
                // ensure TX finished on the line
                uart_wait_tx_done(uart_num, pdMS_TO_TICKS(50));
                vTaskDelay(pdMS_TO_TICKS(1));
                gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 0);
            }
        vTaskDelayUntil(&last_wake, period);
    }
    vTaskDelete(NULL);
}

void start_modbus_master_task()
{
    // Ensure UART is configured for master polling (TX=GPIO32, RX=GPIO33) at requested baud.
    // Note: UART1 cannot be shared by slave+master simultaneously.
    uart_driver_delete(MODBUS_UART_NUM);

    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = MODBUS_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    ESP_ERROR_CHECK(uart_param_config(MODBUS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(MODBUS_UART_NUM, MODBUS_TX_PIN, MODBUS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(MODBUS_UART_NUM, 2048, 0, 0, NULL, 0));

    // configure DE pin if used
    if (MODBUS_DE_PIN >= 0) {
        gpio_reset_pin((gpio_num_t)MODBUS_DE_PIN);
        gpio_set_direction((gpio_num_t)MODBUS_DE_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 0); // default to receive
    }

    xTaskCreate(modbus_master_task, "modbus_master", 4096, NULL, 5, NULL);
}

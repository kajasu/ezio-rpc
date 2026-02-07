#include "modbus_slave.h"
#include "modbus_master.h"
#include "ezapp.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "modbus_slave";

static void log_hex_i(const char *tag, const char *label, const uint8_t *data, int len)
{
    if (!data || len <= 0) return;

    constexpr int BYTES_PER_LINE = 16;
    char line[(BYTES_PER_LINE * 3) + 1];

    for (int i = 0; i < len; i += BYTES_PER_LINE) {
        int chunk = len - i;
        if (chunk > BYTES_PER_LINE) chunk = BYTES_PER_LINE;

        int pos = 0;
        for (int j = 0; j < chunk; ++j) {
            int n = snprintf(line + pos, sizeof(line) - (size_t)pos, "%02X ", (unsigned)data[i + j]);
            if (n <= 0) break;
            pos += n;
            if (pos >= (int)sizeof(line)) {
                pos = (int)sizeof(line) - 1;
                break;
            }
        }
        if (pos > 0) line[pos - 1] = '\0';
        else line[0] = '\0';

        ESP_LOGI(tag, "%s[%d..%d] (%dB): %s", label ? label : "DATA", i, i + chunk - 1, chunk, line);
    }
}

// Track last transmitted RTU frame to filter RS485 echo without
// accidentally dropping legitimate master requests (FC06 request==response).
static uint8_t s_last_tx[260];
static size_t s_last_tx_len = 0;
static int64_t s_last_tx_us = 0;

// Slave UART/settings (copied locally so this file can build standalone)
#ifndef MODBUS_SLAVE_UART_NUM
#define MODBUS_SLAVE_UART_NUM UART_NUM_2
#endif
#ifndef MODBUS_SLAVE_BAUDRATE
#define MODBUS_SLAVE_BAUDRATE 115200
#endif
#ifndef MODBUS_SLAVE_TX_PIN
#define MODBUS_SLAVE_TX_PIN 32
#endif
#ifndef MODBUS_SLAVE_RX_PIN
#define MODBUS_SLAVE_RX_PIN 33
#endif

#ifndef MODBUS_UNIT_ID
#define MODBUS_UNIT_ID 1
#endif

// Optional DE pin (keep default -1 if not used)
#ifndef MODBUS_DE_PIN
#define MODBUS_DE_PIN -1
#endif

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

enum modbus_parse_result_t {
    MODBUS_PARSE_OK = 0,
    MODBUS_PARSE_INCOMPLETE,
    MODBUS_PARSE_INVALID,
};

static modbus_parse_result_t parse_modbus_request_frame(const uint8_t *buf,
                                                       int len,
                                                       uint8_t *out_slave,
                                                       uint8_t *out_func,
                                                       uint16_t *out_addr,
                                                       uint16_t *out_count,
                                                       const uint8_t **out_data,
                                                       int *out_data_len,
                                                       int *out_frame_len)
{
    if (!buf || len <= 0) return MODBUS_PARSE_INCOMPLETE;
    if (len < 2) return MODBUS_PARSE_INCOMPLETE;

    const uint8_t slave = buf[0];
    const uint8_t func = buf[1];
    int expected = 0;
    uint16_t parsed_addr = 0;
    uint16_t parsed_count = 0;
    const uint8_t *parsed_data = nullptr;
    int parsed_data_len = 0;

    if (func == 0x03 || func == 0x04) {
        expected = 8;
        if (len < expected) return MODBUS_PARSE_INCOMPLETE;
        parsed_addr = (uint16_t)((buf[2] << 8) | buf[3]);
        parsed_count = (uint16_t)((buf[4] << 8) | buf[5]);
    } else if (func == 0x10) {
        // request: [slave][0x10][addr_hi][addr_lo][count_hi][count_lo][bytecount][data...][crc_lo][crc_hi]
        if (len < 7) return MODBUS_PARSE_INCOMPLETE;
        const uint16_t count = (uint16_t)((buf[4] << 8) | buf[5]);
        const int bytecount = buf[6];
        if (count == 0 || count > 123) return MODBUS_PARSE_INVALID;
        if (bytecount != (int)count * 2) return MODBUS_PARSE_INVALID;
        expected = 9 + bytecount;
        if (len < expected) return MODBUS_PARSE_INCOMPLETE;

        parsed_addr = (uint16_t)((buf[2] << 8) | buf[3]);
        parsed_count = count;
        parsed_data = &buf[7];
        parsed_data_len = bytecount;
    } else if (func == 0x06) {
        // request: [slave][0x06][addr_hi][addr_lo][value_hi][value_lo][crc_lo][crc_hi]
        expected = 8;
        if (len < expected) return MODBUS_PARSE_INCOMPLETE;
        parsed_addr = (uint16_t)((buf[2] << 8) | buf[3]);
        parsed_count = 1;
        parsed_data = &buf[4];
        parsed_data_len = 2;
    } else {
        return MODBUS_PARSE_INVALID;
    }

    ESP_LOGD(TAG, "parse_modbus: func=0x%02X expected=%d len=%d", func, expected, len);
    const uint16_t recv_crc = (uint16_t)(buf[expected - 2] | (buf[expected - 1] << 8));
    const uint16_t calc_crc = modbus_crc16(buf, (uint16_t)(expected - 2));
    ESP_LOGD(TAG, "CRC check: recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
    if (recv_crc != calc_crc) return MODBUS_PARSE_INVALID;

    *out_slave = slave;
    *out_func = func;
    *out_addr = parsed_addr;
    *out_count = parsed_count;
    *out_frame_len = expected;
    *out_data = parsed_data;
    *out_data_len = parsed_data_len;
    ESP_LOGD(TAG, "parse_modbus OK: slave=%d func=0x%02X addr=%u count=%u", slave, func, *out_addr, *out_count);
    return MODBUS_PARSE_OK;
}

// If RS485 transceiver echoes our own TX back into RX, it can wedge the parser.
// Detect and skip common Modbus RTU *responses* addressed to us.
static bool is_echoed_modbus_response(const uint8_t *buf, int len, uint8_t unit_id, int *out_len)
{
    if (!buf || len < 5) return false;
    if (buf[0] != unit_id) return false;

    const uint8_t func = buf[1];
    if (func == 0x10) {
        // response: [slave][0x10][addr_hi][addr_lo][count_hi][count_lo][crc_lo][crc_hi]
        if (len < 8) return false;
        const uint16_t recv_crc = (uint16_t)(buf[6] | (buf[7] << 8));
        const uint16_t calc_crc = modbus_crc16(buf, 6);
        if (recv_crc != calc_crc) return false;
        *out_len = 8;
        return true;
    }

    if (func == 0x06) {
        // response: identical to request (8 bytes)
        // Only treat as echo if it matches what we *just* transmitted.
        if (len < 8) return false;
        const uint16_t recv_crc = (uint16_t)(buf[6] | (buf[7] << 8));
        const uint16_t calc_crc = modbus_crc16(buf, 6);
        if (recv_crc != calc_crc) return false;
        if (s_last_tx_len != 8) return false;
        if (memcmp(buf, s_last_tx, 8) != 0) return false;
        const int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_tx_us > 5000) return false; // outside typical echo window
        *out_len = 8;
        return true;
    }

    if (func == 0x03 || func == 0x04) {
        // response: [slave][func][bytecount][data...][crc_lo][crc_hi]
        const int bytecount = buf[2];
        const int expected = 3 + bytecount + 2;
        if (len < expected) return false;
        const uint16_t recv_crc = (uint16_t)(buf[expected - 2] | (buf[expected - 1] << 8));
        const uint16_t calc_crc = modbus_crc16(buf, (uint16_t)(expected - 2));
        if (recv_crc != calc_crc) return false;
        *out_len = expected;
        return true;
    }

    return false;
}

static bool uart_write_bytes_blocking(uart_port_t port, const uint8_t *data, size_t len, TickType_t ticks);

static void send_modbus_exception(uart_port_t uart_num, uint8_t unit_id, uint8_t func, uint8_t exception_code)
{
    // response: [slave][func|0x80][exception_code][crc_lo][crc_hi]
    uint8_t resp[5];
    resp[0] = unit_id;
    resp[1] = (uint8_t)(func | 0x80);
    resp[2] = exception_code;
    uint16_t crc = modbus_crc16(resp, 3);
    resp[3] = crc & 0xFF;
    resp[4] = (crc >> 8) & 0xFF;
    uart_write_bytes_blocking(uart_num, resp, sizeof(resp), pdMS_TO_TICKS(200));
}

static bool uart_write_bytes_blocking(uart_port_t port, const uint8_t *data, size_t len, TickType_t ticks)
{
    if (MODBUS_DE_PIN >= 0) {
        gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int written = uart_write_bytes(port, (const char *)data, len);
    if (written < 0) return false;
    uart_wait_tx_done(port, ticks);
    if (MODBUS_DE_PIN >= 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 0);
    }

    // Save last TX frame for echo filtering.
    size_t copy_len = len;
    if (copy_len > sizeof(s_last_tx)) copy_len = sizeof(s_last_tx);
    if (data && copy_len > 0) {
        memcpy(s_last_tx, data, copy_len);
        s_last_tx_len = copy_len;
        s_last_tx_us = esp_timer_get_time();
    } else {
        s_last_tx_len = 0;
        s_last_tx_us = esp_timer_get_time();
    }
    return true;
}

static void modbus_slave_task(void *arg)
{
    (void)arg;
    uart_port_t uart_num = MODBUS_SLAVE_UART_NUM;
    EzApp &app = EzApp::instance();
    const int bufsize = 512;
    uint8_t buf[bufsize];
    int idx = 0;
 
    while (1) {
        int r = uart_read_bytes(uart_num, buf + idx, bufsize - idx, pdMS_TO_TICKS(200));
        if (r > 0) {
            //ESP_LOGI(TAG, "RX %d bytes", r);
            //ESP_LOG_BUFFER_HEXDUMP(TAG, buf + idx, r, ESP_LOG_INFO);
            idx += r;
            int processed = 0;
            for (int offset = 0; offset < idx; ) {
                int rem = idx - offset;
                int echo_len = 0;
                if (is_echoed_modbus_response(buf + offset, rem, MODBUS_UNIT_ID, &echo_len)) {
                    offset += echo_len;
                    processed = offset;
                    continue;
                }

                uint8_t slave = 0, func = 0;
                uint16_t addr = 0, count = 0;
                const uint8_t *data = nullptr;
                int data_len = 0;
                int frame_len = 0;
                modbus_parse_result_t pr = parse_modbus_request_frame(buf + offset,
                                                                     rem,
                                                                     &slave,
                                                                     &func,
                                                                     &addr,
                                                                     &count,
                                                                     &data,
                                                                     &data_len,
                                                                     &frame_len);
                if (pr == MODBUS_PARSE_INCOMPLETE) {
                    break; // wait for more bytes
                }
                if (pr == MODBUS_PARSE_INVALID) {
                    // resync: drop one byte and keep scanning
                    offset += 1;
                    processed = offset;
                    continue;
                }

                //ESP_LOGI(TAG, "Parsed frame: slave=%d func=0x%02X addr=%u count=%u", slave, func, (unsigned)addr, (unsigned)count);


                if (slave == MODBUS_UNIT_ID) {
                    if (func == 0x03 || func == 0x04) {
                        if (count == 0 || count > 125) {
                        } else {
                            int bytecount = count * 2;
                            uint8_t resp[5 + 250];
                            resp[0] = MODBUS_UNIT_ID;
                            resp[1] = func;
                            resp[2] = bytecount;
                            for (int i = 0; i < count; ++i) {
                                int16_t v = 0;
                                uint32_t off = (uint32_t)(addr + i) * 2u;
                                app.readInt16(EzApp::D, off, v);
                                resp[3 + i*2] = (v >> 8) & 0xFF;
                                resp[3 + i*2 + 1] = v & 0xFF;
                            }
                            uint16_t crc = modbus_crc16(resp, 3 + bytecount);
                            resp[3 + bytecount] = crc & 0xFF;
                            resp[3 + bytecount + 1] = (crc >> 8) & 0xFF;
                            //log_hex_i(TAG, "FC03/04 response", resp, 3 + bytecount + 2);
                            uart_write_bytes_blocking(uart_num, resp, 3 + bytecount + 2, pdMS_TO_TICKS(200));
                        }
                    } else if (func == 0x10) {
                        
                        uint16_t count_regs = count;
                        if (count_regs == 0 || count_regs > 123) {
                        } else {
                            for (int i = 0; i < count_regs; ++i) {
                                uint16_t hi = data[i*2];
                                uint16_t lo = data[i*2 + 1];
                                int16_t val = (int16_t)((hi << 8) | lo);
                                uint32_t off = (uint32_t)(addr + i) * 2u;
                                app.writeInt16(EzApp::D, off, val);
                            }
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
                    } else if (func == 0x06) {
                        // Write Single Register
                        ESP_LOGI(TAG, "Write Single Register: addr=%u val=%d ", addr, (int16_t)((data[0] << 8) | data[1]));
                        if (!data || data_len != 2) {
                            send_modbus_exception(uart_num, MODBUS_UNIT_ID, func, 0x03); // Illegal Data Value
                        } else {
                            int16_t val = (int16_t)((data[0] << 8) | data[1]);
                            uint32_t off = (uint32_t)addr * 2u;
                            //ESP_LOGI(TAG, "FC06 Write: addr=%u val=%d off=%u", addr, val, off);
                            if (!app.writeInt16(EzApp::D, off, val)) {
                                send_modbus_exception(uart_num, MODBUS_UNIT_ID, func, 0x02); // Illegal Data Address
                            } else {
                                uint8_t resp[8];
                                resp[0] = MODBUS_UNIT_ID;
                                resp[1] = 0x06;
                                resp[2] = (addr >> 8) & 0xFF;
                                resp[3] = addr & 0xFF;
                                resp[4] = (uint8_t)((val >> 8) & 0xFF);
                                resp[5] = (uint8_t)(val & 0xFF);
                                uint16_t crc = modbus_crc16(resp, 6);
                                resp[6] = crc & 0xFF;
                                resp[7] = (crc >> 8) & 0xFF;
                                uart_write_bytes_blocking(uart_num, resp, 8, pdMS_TO_TICKS(200));
                            }
                        }
                    }
                }

                offset += frame_len;
                processed = offset;
            }
            if (processed > 0) {
                memmove(buf, buf + processed, idx - processed);
                idx -= processed;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void start_modbus_slave_task()
{
    // configure DE pin if used (RS485 direction control)
    if (MODBUS_DE_PIN >= 0) {
        gpio_reset_pin((gpio_num_t)MODBUS_DE_PIN);
        gpio_set_direction((gpio_num_t)MODBUS_DE_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)MODBUS_DE_PIN, 0); // default receive
    }

    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = MODBUS_SLAVE_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    uart_param_config(MODBUS_SLAVE_UART_NUM, &uart_config);
    uart_set_pin(MODBUS_SLAVE_UART_NUM, MODBUS_SLAVE_TX_PIN, MODBUS_SLAVE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MODBUS_SLAVE_UART_NUM, 2048, 0, 0, NULL, 0);

    xTaskCreate(modbus_slave_task, "modbus_slave", 8192, NULL, 5, NULL);
}

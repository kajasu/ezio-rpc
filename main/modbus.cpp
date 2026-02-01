#include "modbus.h"
#include "ezapp.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "modbus";

// UART pins and settings for Modbus RTU slave
#define MODBUS_UART_NUM UART_NUM_1
#define MODBUS_TX_PIN 32
#define MODBUS_RX_PIN 33
#define MODBUS_BAUDRATE 115200
// Slave unit id
#define MODBUS_UNIT_ID 1

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

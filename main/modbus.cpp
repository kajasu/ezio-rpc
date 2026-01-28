#include "modbus.h"
#include "ezapp.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

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

static bool modbus_read_input_registers(uart_port_t port, uint8_t slave, uint16_t addr, uint16_t count, uint16_t *out_regs)
{
    uint8_t req[8];
    req[0] = slave;
    req[1] = 0x04;
    req[2] = (addr >> 8) & 0xFF;
    req[3] = addr & 0xFF;
    req[4] = (count >> 8) & 0xFF;
    req[5] = count & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = (crc >> 8) & 0xFF;

    if (!uart_write_bytes_blocking(port, req, sizeof(req), pdMS_TO_TICKS(200))) return false;

    uint8_t header[3];
    int r = uart_read_bytes(port, header, 3, pdMS_TO_TICKS(500));
    if (r < 3) return false;
    if (header[0] != slave || header[1] != 0x04) return false;
    uint8_t byte_count = header[2];
    if (byte_count != count * 2) return false;
    if (byte_count > 250) return false;
    uint8_t data[256];
    r = uart_read_bytes(port, data, byte_count + 2, pdMS_TO_TICKS(500));
    if (r < byte_count + 2) return false;
    uint8_t resp[3 + 256 + 2];
    resp[0] = header[0]; resp[1] = header[1]; resp[2] = header[2];
    memcpy(&resp[3], data, byte_count + 2);
    uint16_t resp_crc = modbus_crc16(resp, 3 + byte_count);
    uint16_t recv_crc = data[byte_count] | (data[byte_count+1] << 8);
    if (resp_crc != recv_crc) return false;

    for (int i = 0; i < count; ++i) {
        out_regs[i] = (uint16_t)((data[2*i] << 8) | data[2*i+1]);
    }
    return true;
}

static bool modbus_write_multiple_registers(uart_port_t port, uint8_t slave, uint16_t addr, uint16_t count, const uint16_t *regs)
{
    size_t body = 7 + count*2 + 2;
    uint8_t *req = (uint8_t *)malloc(body);
    if (!req) return false;
    req[0] = slave; req[1] = 0x10;
    req[2] = (addr >> 8) & 0xFF; req[3] = addr & 0xFF;
    req[4] = (count >> 8) & 0xFF; req[5] = count & 0xFF;
    req[6] = count * 2;
    for (int i = 0; i < count; ++i) {
        req[7 + i*2] = (regs[i] >> 8) & 0xFF;
        req[7 + i*2 + 1] = regs[i] & 0xFF;
    }
    uint16_t crc = modbus_crc16(req, 7 + count*2);
    req[7 + count*2] = crc & 0xFF;
    req[7 + count*2 + 1] = (crc >> 8) & 0xFF;

    bool ok = uart_write_bytes_blocking(port, req, 7 + count*2 + 2, pdMS_TO_TICKS(500));
    free(req);
    if (!ok) return false;

    uint8_t resp[8];
    int r = uart_read_bytes(port, resp, 8, pdMS_TO_TICKS(1000));
    if (r < 8) return false;
    uint16_t recv_crc = resp[6] | (resp[7] << 8);
    if (modbus_crc16(resp, 6) != recv_crc) return false;
    if (resp[0] != slave || resp[1] != 0x10) return false;
    return true;
}

static void modbus_task(void *arg)
{
    (void)arg;
    uart_port_t uart_num = UART_NUM_1;
    EzApp &app = EzApp::instance();
    const int max_regs = 64;
    uint16_t regs[max_regs];

    while (1) {
        // Read Slave 1 -> store to D at MODBUS_SLAVE1_OFFSET
        if (modbus_read_input_registers(uart_num, 1, 100, 30, regs)) {
            for (int i = 0; i < 30; ++i) app.writeInt16(EzApp::D, EzApp::MODBUS_SLAVE1_OFFSET + i * 2, (int16_t)regs[i]);
        }
        // Read Slave 2 -> store to D at MODBUS_SLAVE2_OFFSET
        if (modbus_read_input_registers(uart_num, 2, 0, 10, regs)) {
            for (int i = 0; i < 10; ++i) app.writeInt16(EzApp::D, EzApp::MODBUS_SLAVE2_OFFSET + i * 2, (int16_t)regs[i]);
        }
        // Prepare regs from D write-source area
        for (int i = 0; i < 30; ++i) { int16_t v = 0; app.readInt16(EzApp::D, EzApp::MODBUS_WRITE_SRC_OFFSET + i * 2, v); regs[i] = (uint16_t)v; }
        modbus_write_multiple_registers(uart_num, 1, 100, 30, regs);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

// configure UART and start task
void start_modbus_task()
{
    // UART init for RS485/Modbus (TX=GPIO27, RX=GPIO14) at 115200
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 27, 14, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);

    xTaskCreate(modbus_task, "modbus", 8192, NULL, 5, NULL);
}

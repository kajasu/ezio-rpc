#include "modbus_master.h"
#include "ezapp.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"

// NVS helpers moved into EzApp methods

static const char *TAG = "modbus";
// Modbus RTU 마스터/슬레이브에 사용하는 UART 핀 및 설정
#define MODBUS_UART_NUM UART_NUM_1
#define MODBUS_TX_PIN 27
#define MODBUS_RX_PIN 14

#define MODBUS_BAUDRATE 19200
// 슬레이브 장치의 유닛 ID (필요시 사용)
#define MODBUS_UNIT_ID 1
#define MODBUS_SLAVE_UART_NUM UART_NUM_2
#define MODBUS_SLAVE_BAUDRATE 115200
#define MODBUS_SLAVE_TX_PIN 32
#define MODBUS_SLAVE_RX_PIN 33


// RS485 트랜시버의 DE(driver enable) 핀(옵션).
// 사용하지 않으면 -1로 설정하십시오.
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

// FC04 (입력 레지스터 읽기) 요청 포맷은 FC03과 동일함
static __attribute__((unused)) int build_modbus_fc04_request(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t *out, int out_size)
{
    // FC03과 동일한 레이아웃: slave(1) func(1) addr(2) count(2) crc(2)
    return build_modbus_fc03_request(slave_id, start_addr, count, out, out_size);
}

// FC16 / 0x10 (다중 레지스터 쓰기) 요청 빌더
// 요청 형식: [slave][0x10][addr_hi][addr_lo][count_hi][count_lo][bytecount][data...][crc_lo][crc_hi]
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
    // 응답 형식: [slave][func][bytecount][data...][crc_lo][crc_hi]
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
        // 작은 단위로 읽어 전체 타임아웃을 보장함(각 호출은 짧게 블록)
        int r = uart_read_bytes(port, dst + got, want - got, pdMS_TO_TICKS(1));
        if (r > 0) got += r;
    }
    return got;
}

static bool uart_write_bytes_blocking(uart_port_t port, const uint8_t *data, size_t len, TickType_t ticks)
{
    int written = uart_write_bytes(port, (const char *)data, len);
    if (written < 0) return false;
    // 전송 완료를 대기
    uart_wait_tx_done(port, ticks);
    return true;
}

static void rs485_set_tx(bool tx_enable)
{
    if (MODBUS_DE_PIN < 0) return;
    gpio_set_level((gpio_num_t)MODBUS_DE_PIN, tx_enable ? 1 : 0);
    // 트랜시버 방향(송수신 전환)이 안정화될 때까지 짧게 대기
    esp_rom_delay_us(50);
}
// Modbus 슬레이브 태스크는 이제 main/modbus_slave.cpp로 이동됨

static void modbus_master_task(void *arg)
{
    (void)arg;
    const uart_port_t uart_num = MODBUS_UART_NUM;

    // 사용자에 의해 요청된 파라미터(폴링 대상 등)
    const uint8_t slave_id = 1;
    const uint16_t start_addr = 1;
    const uint16_t count = 2;
    const TickType_t rx_timeout = pdMS_TO_TICKS(50); // allow more time on RS485/slow slaves
    const TickType_t period_ok = pdMS_TO_TICKS(500);
    const TickType_t period_slow = pdMS_TO_TICKS(2000);

    uint32_t consecutive_timeouts = 0;
    TickType_t cur_period = period_ok;

    ESP_LOGI(TAG, "Modbus master: UART%d TX=%d RX=%d baud=%d slave_id=%u DE=%d",
             (int)uart_num, MODBUS_TX_PIN, MODBUS_RX_PIN, MODBUS_BAUDRATE, (unsigned)slave_id, MODBUS_DE_PIN);

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
        // 수신 버퍼에 남아있는 오래된 바이트를 비움
        uart_flush_input(uart_num);

        // DE 핀을 사용하는 경우 송신 전에 드라이버(트랜시버)를 활성화
        rs485_set_tx(true);
        bool tx_ok = uart_write_bytes_blocking(uart_num, req, req_len, pdMS_TO_TICKS(200));
        rs485_set_tx(false);

        if (!tx_ok) {
            ESP_LOGW(TAG, "Modbus master TX failed");
        } else {
            int got = uart_read_exact(uart_num, resp, resp_len, rx_timeout);
            if (got == resp_len && parse_modbus_fc03_response(resp, got, slave_id, count)) {
                consecutive_timeouts = 0;
                cur_period = period_ok;
                EzApp::instance().setModbusStatus(EzApp::DEVICE_OK);
                int16_t v = ((int16_t)resp[3] << 8) | resp[4];
                int16_t v2 = (((int16_t)resp[5] << 8) | resp[6])*10;
                v *= 10;
                // EzApp offsets are bytes, so D110 => 110*2
                EzApp::instance().writeInt16(EzApp::D, 110 * 2, v);
                int16_t d140_val = 0;
                //ESP_LOGI(TAG, "Modbus master RX success: Reg[%d]=%d,%d", start_addr, v, v2);
                EzApp::instance().readInt16(EzApp::D, 140 * 2, d140_val);
                //
                //ESP_LOGI(TAG, "Modbus master RX success: Reg[%d]=%d,%d", start_addr, v, v2);
                if (v2 != d140_val) {
                    ESP_LOGI(TAG, "Modbus master: D140 changed (D140=%d) (sp=%d)writing new value to slave", d140_val, v2);
                    int16_t write_val = d140_val / 10;
                    uint8_t data[4];
                    data[0] = (write_val >> 8) & 0xFF;
                    data[1] = write_val & 0xFF;
                    uint8_t write_req[16];
                    int write_len = build_modbus_fc10_request(slave_id, 300, 1, data, write_req, sizeof(write_req));
                    if (write_len > 0) {
                        uart_write_bytes_blocking(uart_num, write_req, write_len, pdMS_TO_TICKS(200));
                    }

                    // Persist D130..D159 (30 registers) to NVS so we can restore them on next boot.
                    esp_err_t save_err = EzApp::instance().saveD130RegionToNVS();
                    if (save_err != ESP_OK) {
                        ESP_LOGW(TAG, "Modbus master: failed to persist D130..D159: %s", esp_err_to_name(save_err));
                    }
                }
            } else {
                consecutive_timeouts++;
                // After a few consecutive failures, slow down polling to reduce log spam and bus load.
                if (consecutive_timeouts >= 5) {
                    cur_period = period_slow;
                        EzApp::instance().setModbusStatus(EzApp::DEVICE_ERROR);
                        EzApp::instance().incrementModbusErrorCount();
                }

                // 경고 로깅 제어: 첫 실패는 즉시, 이후는 10번째마다 로깅
                if (consecutive_timeouts == 1 || (consecutive_timeouts % 10) == 0) {
                    if (got > 0) {
                        ESP_LOGW(TAG, "Modbus master RX invalid/timeout (got=%d need=%d, consecutive=%u, errorCount=%u)",
                                 got, resp_len, (unsigned)consecutive_timeouts, (unsigned)EzApp::instance().getModbusErrorCount());
                    } else {
                        ESP_LOGW(TAG, "Modbus master RX timeout (consecutive=%u, errorCount=%u)", 
                                 (unsigned)consecutive_timeouts, (unsigned)EzApp::instance().getModbusErrorCount());
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake, cur_period);
    }
    vTaskDelete(NULL);
}

// D130..D159 persistence is now implemented as EzApp methods.

void start_modbus_master_task()
{
    // 모드버스 마스터 폴링을 위해 UART를 설정(TX=GPIO27, RX=GPIO14, 지정된 baud).
    // 주의: UART1은 마스터/슬레이브 동시에 사용할 수 없음.
    // 시작 전에 이전에 저장된 D130..D159 값을 복원 시도

    uart_driver_delete(MODBUS_UART_NUM);

    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.baud_rate = MODBUS_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    // Leave source_clk at the zero-initialized default to avoid
    // target-specific UART_SCLK_* enum name differences.
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

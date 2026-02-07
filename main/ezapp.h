// EzApp C++ header (renamed from ezapp.hpp)
#ifndef EZAPP_H
#define EZAPP_H

#include <stdint.h>
#ifdef __cplusplus
#include <cstddef>
#include <cstdbool>
// FreeRTOS mutex
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// C++ EzApp class
class EzApp {
public:
    enum Group : int { X = 0, Y = 1, D = 2, COUNT = 3 };

    // Get singleton instance
    static EzApp &instance();

    // Initialize/tear-down (idempotent)
    bool init();
    void deinit();
    
    // Initialize PCF8574 I/O expanders (addresses configured below)
    bool init_pcf8574();

    // RTC (DS1307) support
    // Initialize RTC over I2C (ensures driver is configured). Returns true on success.
    bool init_rtc();

    // Set RTC time. `year` is full year (e.g., 2026). `weekday` is 1..7 (Mon..Sun or device-specific mapping).
    // Returns true on success.
    bool set_rtc_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday = 1);

    // (log API removed)

    // Standardized D-group offsets for peripherals (bytes)
    static constexpr uint32_t MODBUS_D_BASE = 256; // base for Modbus data
    static constexpr uint32_t MODBUS_SLAVE1_OFFSET = MODBUS_D_BASE * 2; // 30 x int16
    static constexpr uint32_t MODBUS_SLAVE2_OFFSET = MODBUS_D_BASE + (30 * 2);
    // Move Modbus write-source region to 512 per request
    static constexpr uint32_t MODBUS_WRITE_SRC_OFFSET = 512; // 30 x int16 source (60 bytes)

    // Restore oxygen offsets to overlap with Modbus write-source (user requested)
    // NOTE: EzApp::writeInt16/readInt16 offsets are byte offsets.
    static constexpr uint32_t OXYGEN_OFFSET1 = 114 * 2; // uint16 at D114 (byte offset)
    static constexpr uint32_t OXYGEN_OFFSET2 = 115 * 2; // uint16 at D115 (byte offset)


    // Size per group
    static constexpr std::size_t GROUP_SIZE = 2048;

    // PCF8574 device addresses (7-bit). Modify if using different addresses.
    static constexpr uint8_t PCF_INPUT_ADDR = 0x22;
    static constexpr uint8_t PCF_OUTPUT_ADDR = 0x24;

    // I2C defaults (change if your hardware uses other pins/port)
    static constexpr int I2C_PORT = 0; // I2C_NUM_0
    // Updated to user-provided pins
    static constexpr int I2C_SDA_IO = 4;
    static constexpr int I2C_SCL_IO = 15;
    static constexpr int I2C_FREQ_HZ = 400000;
    // If true, suppress any warning about using GPIO15 (strapping pin) for SCL
    static constexpr bool I2C_IGNORE_SCL_STRAPPING_WARNING = true;

private:
    EzApp();
    ~EzApp();
    EzApp(const EzApp &) = delete;
    EzApp &operator=(const EzApp &) = delete;

    uint8_t *groups_[COUNT];
    bool initialized_;
    SemaphoreHandle_t mutex_; // protects read/write access to groups_
    
public:
    // --- Single value read/write (byte offsets) ---
    // NOTE: `offset` is a BYTE offset within the selected group.
    bool readByte(Group g, std::size_t offset, uint8_t &out);
    bool writeByte(Group g, std::size_t offset, uint8_t value);

    bool readInt8(Group g, std::size_t offset, int8_t &out);
    bool writeInt8(Group g, std::size_t offset, int8_t value);

    bool readInt16(Group g, std::size_t offset, int16_t &out);
    bool writeInt16(Group g, std::size_t offset, int16_t value);

    bool readInt32(Group g, std::size_t offset, int32_t &out);
    bool writeInt32(Group g, std::size_t offset, int32_t value);

    bool readFloat(Group g, std::size_t offset, float &out);
    bool writeFloat(Group g, std::size_t offset, float value);

    // --- Index helpers (int16 index) ---
    // `index` is an int16 element index (i.e., index 0 => offset 0, index 1 => offset 2).
    bool readInt16AtIndex(Group g, uint32_t index, int16_t &out);
    bool writeInt16AtIndex(Group g, uint32_t index, int16_t value);

    // --- Group read/write (arrays) ---
    // Arrays use BYTE offset; element size is implied by type.
    bool readBytes(Group g, std::size_t offset, void *out, std::size_t len);
    bool writeBytes(Group g, std::size_t offset, const void *data, std::size_t len);

    bool readByteArray(Group g, std::size_t offset, uint8_t *out, std::size_t count);
    bool writeByteArray(Group g, std::size_t offset, const uint8_t *data, std::size_t count);

    bool readInt8Array(Group g, std::size_t offset, int8_t *out, std::size_t count);
    bool writeInt8Array(Group g, std::size_t offset, const int8_t *data, std::size_t count);

    bool readInt16Array(Group g, std::size_t offset, int16_t *out, std::size_t count);
    bool writeInt16Array(Group g, std::size_t offset, const int16_t *data, std::size_t count);

    bool readInt32Array(Group g, std::size_t offset, int32_t *out, std::size_t count);
    bool writeInt32Array(Group g, std::size_t offset, const int32_t *data, std::size_t count);

    bool readFloatArray(Group g, std::size_t offset, float *out, std::size_t count);
    bool writeFloatArray(Group g, std::size_t offset, const float *data, std::size_t count);

    // --- Bit read/write ---
    // Bit addressing is within the group, bit 0 == LSB of byte 0.
    bool readBit(Group g, uint32_t bitIndex, bool &out);
    bool writeBit(Group g, uint32_t bitIndex, bool value);

    // Packed bit array I/O. Bits are packed LSB-first in each output byte.
    // `outPackedBytes` must be at least (bitCount+7)/8.
    bool readBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, uint8_t *outPackedBytes, std::size_t outPackedLen);
    bool writeBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, const uint8_t *packedBytes, std::size_t packedLen);
};

#endif // __cplusplus

#endif // EZAPP_H

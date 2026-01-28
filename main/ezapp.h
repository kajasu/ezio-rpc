// EzApp C++ header (renamed from ezapp.hpp)
#ifndef EZAPP_H
#define EZAPP_H

#include <cstdint>
#include <cstddef>

class EzApp {
public:
    enum Group : int { X = 0, Y = 1, D = 2, COUNT = 3 };

    // Get singleton instance
    static EzApp &instance();

    // Initialize/tear-down (idempotent)
    bool init();
    void deinit();

    // Typed read/write. Return 0 on success, -1 on error.
    int writeShort(Group g, uint32_t offset, short value);
    int readShort(Group g, uint32_t offset, short &out);

    int writeInt16(Group g, uint32_t offset, int16_t value);
    int readInt16(Group g, uint32_t offset, int16_t &out);

    int writeInt32(Group g, uint32_t offset, int32_t value);
    int readInt32(Group g, uint32_t offset, int32_t &out);

    int writeFloat(Group g, uint32_t offset, float value);
    int readFloat(Group g, uint32_t offset, float &out);

    // Size per group
    static constexpr std::size_t GROUP_SIZE = 4096;

private:
    EzApp();
    ~EzApp();
    EzApp(const EzApp &) = delete;
    EzApp &operator=(const EzApp &) = delete;

    uint8_t *groups_[COUNT];
    bool initialized_;
};

#endif // EZAPP_H

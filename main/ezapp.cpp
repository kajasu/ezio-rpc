#include "ezapp.h"
#include <cstring>
#include <cstdlib>
#include <mutex>

EzApp &EzApp::instance()
{
    static EzApp inst;
    return inst;
}

EzApp::EzApp()
    : initialized_(false)
{
    for (int i = 0; i < COUNT; ++i) groups_[i] = nullptr;
}

EzApp::~EzApp()
{
    deinit();
}

bool EzApp::init()
{
    if (initialized_) return true;

    for (int i = 0; i < COUNT; ++i) {
        groups_[i] = static_cast<uint8_t *>(std::malloc(GROUP_SIZE));
        if (!groups_[i]) {
            for (int j = 0; j < i; ++j) std::free(groups_[j]);
            return false;
        }
        std::memset(groups_[i], 0, GROUP_SIZE);
    }
    initialized_ = true;
    return true;
}

void EzApp::deinit()
{
    if (!initialized_) return;
    for (int i = 0; i < COUNT; ++i) {
        std::free(groups_[i]);
        groups_[i] = nullptr;
    }
    initialized_ = false;
}

static inline int check_bounds_cpp(EzApp::Group g, uint32_t offset, uint32_t size)
{
    if (g < EzApp::X || g >= EzApp::COUNT) return -1;
    if (offset + size > EzApp::GROUP_SIZE) return -1;
    if (!EzApp::instance().init()) return -1;
    return 0;
}

int EzApp::writeShort(Group g, uint32_t offset, short value)
{
    if (check_bounds_cpp(g, offset, sizeof(short)) != 0) return -1;
    std::memcpy(&groups_[g][offset], &value, sizeof(short));
    return 0;
}

int EzApp::readShort(Group g, uint32_t offset, short &out)
{
    if (check_bounds_cpp(g, offset, sizeof(short)) != 0) return -1;
    std::memcpy(&out, &groups_[g][offset], sizeof(short));
    return 0;
}

int EzApp::writeInt16(Group g, uint32_t offset, int16_t value)
{
    if (check_bounds_cpp(g, offset, sizeof(int16_t)) != 0) return -1;
    std::memcpy(&groups_[g][offset], &value, sizeof(int16_t));
    return 0;
}

int EzApp::readInt16(Group g, uint32_t offset, int16_t &out)
{
    if (check_bounds_cpp(g, offset, sizeof(int16_t)) != 0) return -1;
    std::memcpy(&out, &groups_[g][offset], sizeof(int16_t));
    return 0;
}

int EzApp::writeInt32(Group g, uint32_t offset, int32_t value)
{
    if (check_bounds_cpp(g, offset, sizeof(int32_t)) != 0) return -1;
    std::memcpy(&groups_[g][offset], &value, sizeof(int32_t));
    return 0;
}

int EzApp::readInt32(Group g, uint32_t offset, int32_t &out)
{
    if (check_bounds_cpp(g, offset, sizeof(int32_t)) != 0) return -1;
    std::memcpy(&out, &groups_[g][offset], sizeof(int32_t));
    return 0;
}

int EzApp::writeFloat(Group g, uint32_t offset, float value)
{
    if (check_bounds_cpp(g, offset, sizeof(float)) != 0) return -1;
    std::memcpy(&groups_[g][offset], &value, sizeof(float));
    return 0;
}

int EzApp::readFloat(Group g, uint32_t offset, float &out)
{
    if (check_bounds_cpp(g, offset, sizeof(float)) != 0) return -1;
    std::memcpy(&out, &groups_[g][offset], sizeof(float));
    return 0;
}

// C compatibility wrappers
extern "C" {
int EzApp_init(void)
{
    return EzApp::instance().init() ? 0 : -1;
}

void EzApp_deinit(void)
{
    EzApp::instance().deinit();
}

int EzApp_write_short(EzGroup group, uint32_t offset, short value)
{
    return EzApp::instance().writeShort(static_cast<EzApp::Group>(group), offset, value);
}

int EzApp_read_short(EzGroup group, uint32_t offset, short *out_value)
{
    if (!out_value) return -1;
    short tmp = 0;
    int r = EzApp::instance().readShort(static_cast<EzApp::Group>(group), offset, tmp);
    if (r == 0) *out_value = tmp;
    return r;
}

int EzApp_write_int16(EzGroup group, uint32_t offset, int16_t value)
{
    return EzApp::instance().writeInt16(static_cast<EzApp::Group>(group), offset, value);
}

int EzApp_read_int16(EzGroup group, uint32_t offset, int16_t *out_value)
{
    if (!out_value) return -1;
    int16_t tmp = 0;
    int r = EzApp::instance().readInt16(static_cast<EzApp::Group>(group), offset, tmp);
    if (r == 0) *out_value = tmp;
    return r;
}

int EzApp_write_int32(EzGroup group, uint32_t offset, int32_t value)
{
    return EzApp::instance().writeInt32(static_cast<EzApp::Group>(group), offset, value);
}

int EzApp_read_int32(EzGroup group, uint32_t offset, int32_t *out_value)
{
    if (!out_value) return -1;
    int32_t tmp = 0;
    int r = EzApp::instance().readInt32(static_cast<EzApp::Group>(group), offset, tmp);
    if (r == 0) *out_value = tmp;
    return r;
}

int EzApp_write_float(EzGroup group, uint32_t offset, float value)
{
    return EzApp::instance().writeFloat(static_cast<EzApp::Group>(group), offset, value);
}

int EzApp_read_float(EzGroup group, uint32_t offset, float *out_value)
{
    if (!out_value) return -1;
    float tmp = 0.0f;
    int r = EzApp::instance().readFloat(static_cast<EzApp::Group>(group), offset, tmp);
    if (r == 0) *out_value = tmp;
    return r;
}

} // extern "C"

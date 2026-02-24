#pragma once
#include <stdint.h>

// 공통 유틸리티 함수 모음
class Utils
{
public:
    Utils() = delete; // 인스턴스 생성 불가 (정적 메서드 전용 클래스)

    // BCD(이진화 십진수) → 10진수 변환
    // 예: 0x39 → 39
    static inline uint8_t from_bcd(uint8_t bcd)
    {
        return static_cast<uint8_t>((bcd >> 4u) * 10u + (bcd & 0x0Fu));
    }

    // 10진수 → BCD(이진화 십진수) 변환
    // 예: 39 → 0x39  (0~99 범위)
    static inline uint8_t to_bcd(uint8_t dec)
    {
        return static_cast<uint8_t>(((dec / 10u) << 4u) | (dec % 10u));
    }
};

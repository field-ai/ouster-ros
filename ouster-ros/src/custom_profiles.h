#pragma once

#include <string>
#include <utility>
#include <vector>

#include "ouster/chanfield.h"
#include "ouster/impl/profile_extension.h"

namespace ouster_ros {

static constexpr int CUSTOM_RGB_PROFILE_NR = 22;

inline void register_custom_profiles() {
    static bool registered = false;
    if (registered) return;
    registered = true;

    using ouster::sdk::core::ChanField;
    using ouster::sdk::core::ChanFieldType;
    using ouster::sdk::core::impl::FieldInfo;

    // OS-0-128-RGB prototype sensor custom UDP profile (ID 22)
    // 20 bytes/pixel: RANGE(19b) FLAGS(5b) REFL(8b) SIG(16b) NIR(16b)
    //                 R(16b) G(16b) B(16b) R-ATTEN(16b) G-ATTEN(16b) B-ATTEN(16b)
    std::vector<std::pair<std::string, FieldInfo>> fields{
        {ChanField::RANGE,        {ChanFieldType::UINT32, 0,  0x0007FFFF, 0}},
        {ChanField::FLAGS,        {ChanFieldType::UINT8,  2,  0xF8,       3}},
        {ChanField::REFLECTIVITY, {ChanFieldType::UINT8,  3,  0,          0}},
        {ChanField::SIGNAL,       {ChanFieldType::UINT16, 4,  0,          0}},
        {ChanField::NEAR_IR,      {ChanFieldType::UINT16, 6,  0,          0}},
        {ChanField::R,            {ChanFieldType::UINT32, 8,  0xFFFF,     0}},
        {ChanField::G,            {ChanFieldType::UINT32, 10, 0xFFFF,     0}},
        {ChanField::B,            {ChanFieldType::UINT32, 12, 0xFFFF,     0}},
        {ChanField::RAW32_WORD1,  {ChanFieldType::UINT32, 0,  0,          0}},
        {ChanField::RAW32_WORD2,  {ChanFieldType::UINT32, 4,  0,          0}},
        {ChanField::RAW32_WORD3,  {ChanFieldType::UINT32, 8,  0,          0}},
        {ChanField::RAW32_WORD4,  {ChanFieldType::UINT32, 12, 0,          0}},
        {ChanField::RAW32_WORD5,  {ChanFieldType::UINT32, 16, 0,          0}},
    };

    ouster::sdk::core::add_custom_profile(
        CUSTOM_RGB_PROFILE_NR,
        "RNG19_RFL8_SIG16_NIR16_R16_G16_B16",
        fields,
        20);
}

}  // namespace ouster_ros

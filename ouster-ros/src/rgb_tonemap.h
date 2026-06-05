/**
 * @file rgb_tonemap.h
 * @brief Offline RGB tone-mapping for native-RGB lidar profiles.
 *
 * The SDK delivers RGB as raw 16-bit R/G/B channels, but the point cloud
 * composition (color_point / native RGB point types) reads 8-bit R8/G8/B8
 * channels. The live driver derives R8/G8/B8 from R/G/B via AutoExposure in
 * lidar_packet_handler.h; the offline converters (pcap_to_mcap,
 * offline_packet_converter_node) have no packet handler, so they use this
 * helper to perform the identical conversion before composing the cloud.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <memory>

#include <Eigen/Core>
#include <ouster/image_processing.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>

namespace ouster_ros {

namespace rgb_tonemap_detail {

// Fast float16 -> float32 for normal-range values (mirrors lidar_packet_handler.h).
inline float f16_bits_to_f32(uint16_t bits) {
  if (bits == 0) return 0.0f;
  const uint32_t expanded = static_cast<uint32_t>(bits + 0x1C000u) << 13;
  float result;
  std::memcpy(&result, &expanded, sizeof(float));
  return result;
}

inline uint8_t f32_to_u8(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

}  // namespace rgb_tonemap_detail

/**
 * @brief Adds and populates R8/G8/B8 channels for RGB16 lidar profiles.
 *
 * Construct once per stream, call setup() on the (re-used) scan before
 * batching, then call apply() on each completed scan before composing the
 * point cloud. A no-op for non-RGB profiles, so it is safe to wire in
 * unconditionally.
 */
class RgbTonemapper {
 public:
  // Detects RGB16/RGB16_DUAL profiles and, if matched, allocates the
  // auto-exposure state and adds the R8/G8/B8 fields to `scan`.
  void setup(ouster::sdk::core::LidarScan& scan,
             const ouster::sdk::core::SensorInfo& info) {
    namespace core = ouster::sdk::core;
    const auto profile = info.format.udp_profile_lidar;
    has_rgb_ = profile == core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 ||
               profile == core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL;
    if (!has_rgb_) return;

    using core::fd_array;
    using core::img_t;
    r_field_float_ = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
    g_field_float_ = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
    b_field_float_ = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
    auto_exposure_ = std::make_unique<core::image::AutoExposure>();
    scan.add_field(core::ChanField::R8, fd_array<uint8_t>(scan.h, scan.w));
    scan.add_field(core::ChanField::G8, fd_array<uint8_t>(scan.h, scan.w));
    scan.add_field(core::ChanField::B8, fd_array<uint8_t>(scan.h, scan.w));
  }

  // Derives R8/G8/B8 from the raw 16-bit R/G/B channels the batcher populated,
  // applying auto-exposure tone-mapping. No-op for non-RGB profiles.
  void apply(ouster::sdk::core::LidarScan& scan) {
    if (!has_rgb_) return;
    namespace core = ouster::sdk::core;
    using core::img_t;
    using rgb_tonemap_detail::f16_bits_to_f32;
    using rgb_tonemap_detail::f32_to_u8;

    Eigen::Ref<img_t<uint16_t>> r_field = scan.field<uint16_t>(core::ChanField::R);
    Eigen::Ref<img_t<uint16_t>> g_field = scan.field<uint16_t>(core::ChanField::G);
    Eigen::Ref<img_t<uint16_t>> b_field = scan.field<uint16_t>(core::ChanField::B);

    const uint16_t* r_in = r_field.data();
    const uint16_t* g_in = g_field.data();
    const uint16_t* b_in = b_field.data();
    float* r_out = r_field_float_.data();
    float* g_out = g_field_float_.data();
    float* b_out = b_field_float_.data();
    for (Eigen::Index i = 0; i < r_field.size(); ++i) {
      r_out[i] = f16_bits_to_f32(r_in[i]);
      g_out[i] = f16_bits_to_f32(g_in[i]);
      b_out[i] = f16_bits_to_f32(b_in[i]);
    }

    auto_exposure_->update(r_field_float_, g_field_float_, b_field_float_, true);

    Eigen::Ref<img_t<uint8_t>> r8 = scan.field<uint8_t>(core::ChanField::R8);
    Eigen::Ref<img_t<uint8_t>> g8 = scan.field<uint8_t>(core::ChanField::G8);
    Eigen::Ref<img_t<uint8_t>> b8 = scan.field<uint8_t>(core::ChanField::B8);
    uint8_t* r8_out = r8.data();
    uint8_t* g8_out = g8.data();
    uint8_t* b8_out = b8.data();
    for (Eigen::Index i = 0; i < r8.size(); ++i) {
      r8_out[i] = f32_to_u8(r_out[i]);
      g8_out[i] = f32_to_u8(g_out[i]);
      b8_out[i] = f32_to_u8(b_out[i]);
    }
  }

  bool has_rgb() const { return has_rgb_; }

 private:
  bool has_rgb_ = false;
  ouster::sdk::core::img_t<float> r_field_float_;
  ouster::sdk::core::img_t<float> g_field_float_;
  ouster::sdk::core::img_t<float> b_field_float_;
  std::unique_ptr<ouster::sdk::core::image::AutoExposure> auto_exposure_;
};

}  // namespace ouster_ros

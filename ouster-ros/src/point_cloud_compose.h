#pragma once

#include <cmath>
#include <cstring>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_ros/os_point.h"
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster/typedefs.h"
#include "ouster/chanfield.h"

#include "point_meta_helpers.h"
#include "point_transform.h"

namespace ouster_ros {

namespace impl {

inline float f16_bits_to_f32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exponent = (h >> 10) & 0x1F;
    uint32_t mantissa = h & 0x3FF;
    float result;
    if (exponent == 0) {
        if (mantissa == 0) {
            uint32_t bits = sign << 31;
            std::memcpy(&result, &bits, sizeof(result));
        } else {
            result = std::ldexp(static_cast<float>(mantissa), -24);
            if (sign) result = -result;
        }
    } else if (exponent == 0x1F) {
        if (mantissa == 0) {
            result = sign ? -std::numeric_limits<float>::infinity()
                          : std::numeric_limits<float>::infinity();
        } else {
            result = std::numeric_limits<float>::quiet_NaN();
        }
    } else {
        uint32_t bits = (sign << 31) |
                        ((exponent - 15 + 127) << 23) |
                        (mantissa << 13);
        std::memcpy(&result, &bits, sizeof(result));
    }
    return result;
}

inline uint8_t f16_rgb_to_u8(uint16_t h) {
    float f = f16_bits_to_f32(h);
    float clamped = std::fmin(std::fmax(f * 255.0f, 0.0f), 255.0f);
    return static_cast<uint8_t>(clamped + 0.5f);
}

}  // namespace impl

using ouster::sdk::core::ChanFieldType;

template <ChanFieldType T>
struct TypeSelector { /*undefined*/
};

template <>
struct TypeSelector<ChanFieldType::UINT8> {
    typedef uint8_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT16> {
    typedef uint16_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT32> {
    typedef uint32_t type;
};

template <>
struct TypeSelector<ChanFieldType::UINT64> {
    typedef uint64_t type;
};

/**
 * @brief constructs a suitable tuple at compile time that can store a reference
 * to all the fields of a specific LidarScan object (without conversion)
 * according to the information specificed by the ChanFieldTable.
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table>
constexpr auto make_lidar_scan_tuple() {
    if constexpr (Index < N) {
        using ElementType = typename TypeSelector<Table[Index].second>::type;
        return std::tuple_cat(
            std::make_tuple(static_cast<const ElementType*>(0)),
            std::move(make_lidar_scan_tuple<Index + 1, N, Table>()));
    } else {
        return std::make_tuple();
    }
}

/**
 * @brief maps the fields of a LidarScan object to the elements of the supplied
 * tuple in the same order.
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table,
          typename Tuple>
void map_lidar_scan_fields_to_tuple(Tuple& tp, const ouster::sdk::core::LidarScan& ls) {
    static_assert(
        std::tuple_size_v<Tuple> == N,
        "target tuple size has a different size from the channel field table");
    if constexpr (Index < N) {
        using FieldType = typename TypeSelector<Table[Index].second>::type;
        using ElementType = std::remove_const_t<
            std::remove_pointer_t<std::tuple_element_t<Index, Tuple>>>;
        static_assert(std::is_same_v<ElementType, FieldType>,
                      "tuple element, field element types mismatch!");
        std::get<Index>(tp) = ls.field<FieldType>(Table[Index].first).data();
        map_lidar_scan_fields_to_tuple<Index + 1, N, Table>(tp, ls);
    }
}

/**
 * @brief constructs a suitable tuple at compile time that can store a reference
 * to all the fields of a specific LidarScan object (without conversion)
 * according to the information specificed by the ChanFieldTable and directly
 * maps the fields of the supplied LidarScan to the constructed tuple before
 * returning.
 * @param[in] ls LidarScan
 */
template <std::size_t Index, std::size_t N, const ChanFieldTable<N>& Table>
constexpr auto make_lidar_scan_tuple(const ouster::sdk::core::LidarScan& ls) {
    auto tp = make_lidar_scan_tuple<0, N, Table>();
    map_lidar_scan_fields_to_tuple<0, N, Table>(tp, ls);
    return tp;
}

/**
 * @brief copies field values from LidarScan fields combined as a tuple into the
 * the corresponding elements of the input point pt.
 * @param[out] pt point to copy values into.
 * @param[in] tp tuple containing arrays to copy LidarScan field values from.
 * @param[in] idx index of the point to be copied.
 * @remark this method is to be used mainly with sensor native point types.
 */
template <std::size_t Index, typename PointT, typename Tuple>
void copy_lidar_scan_fields_to_point(PointT& pt, const Tuple& tp, int idx) {
    if constexpr (Index < std::tuple_size_v<Tuple>) {
        point::get<5 + Index>(pt) = std::get<Index>(tp)[idx];
        copy_lidar_scan_fields_to_point<Index + 1>(pt, tp, idx);
    } else {
        unused_variable(pt);
        unused_variable(tp);
        unused_variable(idx);
    }
}

template <class T>
using Cloud = pcl::PointCloud<T>;

// TODO[UN]: make this a functor
template <std::size_t N, const ChanFieldTable<N>& PROFILE, typename PointT,
          typename PointS>
void scan_to_cloud_f(ouster_ros::Cloud<PointT>& cloud, PointS& staging_point,
                     const ouster::sdk::core::PointCloudXYZf& points, uint64_t scan_ts,
                     const ouster::sdk::core::LidarScan& ls,
                     const std::vector<int>& pixel_shift_by_row,
                     bool organized = false, bool destagger = true,
                     int rows_step = 1) {
    constexpr bool handle_rgb = point::has_rgb_v<PointS>;

    auto ls_tuple = make_lidar_scan_tuple<0, N, PROFILE>(ls);
    auto timestamp = ls.timestamp();

    const ouster::sdk::core::float16_t* rgb_data = nullptr;
    if constexpr (handle_rgb) {
        try {
            const auto& rgb_field = ls.field(ouster::sdk::core::ChanField::RGB);
            rgb_data = rgb_field.template get<ouster::sdk::core::float16_t>();
        } catch (...) {
            rgb_data = nullptr;
        }
    }

    if (!organized) cloud.clear();
    cloud.is_dense = true;

    int h = static_cast<int>(ls.h);
    int w = static_cast<int>(ls.w);

    for (auto u = 0; u < h; u += rows_step) {
        for (auto v = 0; v < w; ++v) {
            const auto v_shift =
                destagger ? (v + w - pixel_shift_by_row[u]) % w : v;
            const auto src_idx = u * w + v_shift;
            const auto xyz = points.row(src_idx);
            const auto tgt_idx =
                organized ? (u / rows_step) * w + v : cloud.size();

            auto ts =
                timestamp[v_shift] > scan_ts ? timestamp[v_shift] - scan_ts : 0UL;

            if (organized) {
                cloud.is_dense &= !xyz.hasNaN();
            } else {
                if (xyz.hasNaN())
                    continue;
                else
                    cloud.points.emplace_back();
            }

            auto& pt = CondBinaryBind<std::is_same_v<PointT, PointS>>::run(
                cloud.points[tgt_idx], staging_point);
            pt.x = static_cast<decltype(pt.x)>(xyz(0));
            pt.y = static_cast<decltype(pt.y)>(xyz(1));
            pt.z = static_cast<decltype(pt.z)>(xyz(2));
            pt.t = static_cast<uint32_t>(ts);
            pt.ring = static_cast<uint16_t>(u);
            copy_lidar_scan_fields_to_point<0>(pt, ls_tuple, src_idx);

            if constexpr (handle_rgb) {
                if (rgb_data) {
                    pt.r = impl::f16_rgb_to_u8(rgb_data[src_idx * 3 + 0].data);
                    pt.g = impl::f16_rgb_to_u8(rgb_data[src_idx * 3 + 1].data);
                    pt.b = impl::f16_rgb_to_u8(rgb_data[src_idx * 3 + 2].data);
                } else {
                    pt.r = 0; pt.g = 0; pt.b = 0;
                }
            }

            CondBinaryOp<!std::is_same_v<PointT, PointS>>::run(
                cloud.points[tgt_idx], staging_point,
                [](auto& tgt_pt, const auto& src_pt) {
                    point::transform(tgt_pt, src_pt);
                });
            if constexpr (::has_column_v<PointT>) {
                cloud.points[tgt_idx].column = static_cast<uint16_t>(v);
            }
        }
    }
}

}  // namespace ouster_ros
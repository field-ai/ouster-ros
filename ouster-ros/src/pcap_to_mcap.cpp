// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ouster/image_processing.h>
#include <ouster/lidar_scan.h>
#include <ouster/os_pcap.h>
#include <ouster/types.h>

#include "imu_packet_handler.h"
#include "point_cloud_processor_factory.h"

#include <getopt.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ChanField = ouster::sdk::core::ChanField;

namespace {

constexpr size_t MAX_BAGFILE_SIZE_BYTES = 512ULL * 1024ULL * 1024ULL;  // 512 MiB, matches pcap.zst rotation

// RGB tone-mapping helpers. These mirror lidar_packet_handler.h so that the
// offline pcap conversion produces the same R8/G8/B8 channels (and therefore
// the same packed `rgb` point field) that the live driver produces.
inline float rgb_f16_bits_to_f32(uint16_t bits) {
  if (bits == 0) return 0.0f;
  const uint32_t expanded = static_cast<uint32_t>(bits + 0x1C000u) << 13;
  float result;
  std::memcpy(&result, &expanded, sizeof(float));
  return result;
}

inline uint8_t rgb_f32_to_u8(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

struct Args {
  std::vector<std::string> pcaps;
  std::string metadata;
  std::string output_bag;
  std::string robot_namespace;
  std::string point_type = "original";
  bool organized = true;
  bool destagger = true;
  double min_range = 0.0;
  double max_range = 1000.0;
  int v_reduction = 1;
  std::string mask_path;
  std::string timestamp_mode = "TIME_FROM_PTP_1588";
  int64_t ptp_utc_tai_offset = 0;
};

void print_usage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " \\\n"
      << "    --pcap <file> [--pcap <file> ...] \\\n"
      << "    --metadata <ouster_metadata.json> \\\n"
      << "    --output-bag <dir> \\\n"
      << "    --robot-namespace <name> \\\n"
      << "    [--point-type original] [--organized 0|1] [--destagger 0|1] \\\n"
      << "    [--min-range 0.0] [--max-range 1000.0] [--v-reduction 1] \\\n"
      << "    [--mask-path <file>] [--timestamp-mode TIME_FROM_PTP_1588]\n";
}

bool parse_bool(const std::string& s) { return s == "1" || s == "true" || s == "True"; }

bool parse_args(int argc, char** argv, Args& out) {
  static const struct option long_opts[] = {
      {"pcap", required_argument, nullptr, 'p'},
      {"metadata", required_argument, nullptr, 'm'},
      {"output-bag", required_argument, nullptr, 'o'},
      {"robot-namespace", required_argument, nullptr, 'n'},
      {"point-type", required_argument, nullptr, 't'},
      {"organized", required_argument, nullptr, 'g'},
      {"destagger", required_argument, nullptr, 'd'},
      {"min-range", required_argument, nullptr, 'i'},
      {"max-range", required_argument, nullptr, 'x'},
      {"v-reduction", required_argument, nullptr, 'v'},
      {"mask-path", required_argument, nullptr, 'k'},
      {"timestamp-mode", required_argument, nullptr, 's'},
      {"ptp-utc-tai-offset", required_argument, nullptr, 'u'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0},
  };
  int c;
  while ((c = getopt_long(argc, argv, "", long_opts, nullptr)) != -1) {
    switch (c) {
      case 'p': out.pcaps.emplace_back(optarg); break;
      case 'm': out.metadata = optarg; break;
      case 'o': out.output_bag = optarg; break;
      case 'n': out.robot_namespace = optarg; break;
      case 't': out.point_type = optarg; break;
      case 'g': out.organized = parse_bool(optarg); break;
      case 'd': out.destagger = parse_bool(optarg); break;
      case 'i': out.min_range = std::stod(optarg); break;
      case 'x': out.max_range = std::stod(optarg); break;
      case 'v': out.v_reduction = std::stoi(optarg); break;
      case 'k': out.mask_path = optarg; break;
      case 's': out.timestamp_mode = optarg; break;
      case 'u': out.ptp_utc_tai_offset = std::stoll(optarg); break;
      case 'h': print_usage(argv[0]); return false;
      default: print_usage(argv[0]); return false;
    }
  }
  if (out.pcaps.empty() || out.metadata.empty() || out.output_bag.empty() ||
      out.robot_namespace.empty()) {
    print_usage(argv[0]);
    return false;
  }
  return true;
}

ouster::sdk::core::SensorInfo load_metadata(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("Cannot open metadata file: " + path);
  std::stringstream buf;
  buf << ifs.rdbuf();
  return ouster::sdk::core::SensorInfo(buf.str());
}

uint64_t extract_scan_timestamp(const ouster::sdk::core::LidarScan& scan, uint64_t fallback) {
  auto ts = scan.timestamp();
  auto it = std::find_if(ts.data(), ts.data() + ts.size(), [](uint64_t t) { return t != 0; });
  return (it != ts.data() + ts.size()) ? *it : fallback;
}

template <typename Msg>
std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialize(
    const Msg& msg, const std::string& topic, uint64_t stamp_ns) {
  rclcpp::Serialization<Msg> ser;
  auto serialized = std::make_shared<rclcpp::SerializedMessage>();
  ser.serialize_message(&msg, serialized.get());
  auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_msg->topic_name = topic;
  bag_msg->serialized_data =
      std::shared_ptr<rcutils_uint8_array_t>(serialized, &serialized->get_rcl_serialized_message());
  bag_msg->recv_timestamp = static_cast<rcutils_time_point_value_t>(stamp_ns);
  return bag_msg;
}

struct BatchState {
  ouster::sdk::core::ScanBatcher batcher;
  ouster::sdk::core::LidarScan scan;
  ouster::sdk::core::LidarPacket lidar_packet;
  ouster::sdk::core::ImuPacket imu_packet;
  bool is_first_scan = true;

  // RGB tone-mapping state (only used for RGB16 profiles).
  bool has_rgb = false;
  ouster::sdk::core::img_t<float> r_field_float;
  ouster::sdk::core::img_t<float> g_field_float;
  ouster::sdk::core::img_t<float> b_field_float;
  std::unique_ptr<ouster::sdk::core::image::AutoExposure> auto_exposure;

  BatchState(const ouster::sdk::core::SensorInfo& info,
             const ouster::sdk::core::PacketFormat& pf)
      : batcher(info),
        // NOTE: construct with SensorInfo (not the (w, h, profile) overload) so
        // that the RGB profile's channels are laid out the same way the live
        // driver lays them out (see lidar_packet_handler.h).
        scan(info),
        lidar_packet(pf.lidar_packet_size),
        imu_packet(pf.imu_packet_size) {
    auto packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(pf);
    lidar_packet.format = packet_format;
    imu_packet.format = packet_format;

    const auto profile = info.format.udp_profile_lidar;
    if (profile == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 ||
        profile == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL) {
      has_rgb = true;
      using ouster::sdk::core::fd_array;
      using ouster::sdk::core::img_t;
      r_field_float = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
      g_field_float = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
      b_field_float = img_t<float>(info.format.columns_per_frame, info.format.pixels_per_column);
      auto_exposure = std::make_unique<ouster::sdk::core::image::AutoExposure>();
      scan.add_field(ChanField::R8, fd_array<uint8_t>(scan.h, scan.w));
      scan.add_field(ChanField::G8, fd_array<uint8_t>(scan.h, scan.w));
      scan.add_field(ChanField::B8, fd_array<uint8_t>(scan.h, scan.w));
    }
  }

  // Derive the 8-bit R8/G8/B8 channels (consumed by the point cloud processor)
  // from the raw 16-bit R/G/B channels the batcher populates, applying the same
  // auto-exposure tone-mapping as the live driver. No-op for non-RGB profiles.
  void apply_rgb_tonemap() {
    if (!has_rgb) return;
    using ouster::sdk::core::img_t;
    Eigen::Ref<img_t<uint16_t>> r_field = scan.field<uint16_t>(ChanField::R);
    Eigen::Ref<img_t<uint16_t>> g_field = scan.field<uint16_t>(ChanField::G);
    Eigen::Ref<img_t<uint16_t>> b_field = scan.field<uint16_t>(ChanField::B);

    const uint16_t* r_in = r_field.data();
    const uint16_t* g_in = g_field.data();
    const uint16_t* b_in = b_field.data();
    float* r_out = r_field_float.data();
    float* g_out = g_field_float.data();
    float* b_out = b_field_float.data();
    for (Eigen::Index i = 0; i < r_field.size(); ++i) {
      r_out[i] = rgb_f16_bits_to_f32(r_in[i]);
      g_out[i] = rgb_f16_bits_to_f32(g_in[i]);
      b_out[i] = rgb_f16_bits_to_f32(b_in[i]);
    }

    auto_exposure->update(r_field_float, g_field_float, b_field_float, true);

    Eigen::Ref<img_t<uint8_t>> r8 = scan.field<uint8_t>(ChanField::R8);
    Eigen::Ref<img_t<uint8_t>> g8 = scan.field<uint8_t>(ChanField::G8);
    Eigen::Ref<img_t<uint8_t>> b8 = scan.field<uint8_t>(ChanField::B8);
    uint8_t* r8_out = r8.data();
    uint8_t* g8_out = g8.data();
    uint8_t* b8_out = b8.data();
    for (Eigen::Index i = 0; i < r8.size(); ++i) {
      r8_out[i] = rgb_f32_to_u8(r_out[i]);
      g8_out[i] = rgb_f32_to_u8(g_out[i]);
      b8_out[i] = rgb_f32_to_u8(b_out[i]);
    }
  }
};

using ScanSink = std::function<void(const ouster::sdk::core::LidarScan&, uint64_t)>;
using ImuSink = std::function<void(const ouster::sdk::core::ImuPacket&)>;

void process_pcap(const std::string& pcap_path,
                  const ouster::sdk::core::SensorInfo& info,
                  const ouster::sdk::core::PacketFormat& pf, BatchState& state,
                  const ScanSink& on_scan, const ImuSink& on_imu,
                  const rclcpp::Logger& logger) {
  RCLCPP_INFO(logger, "Processing pcap: %s", pcap_path.c_str());
  ouster::sdk::pcap::PcapReader pcap(pcap_path);
  size_t payload_size = pcap.next_packet();
  while (payload_size && rclcpp::ok()) {
    const auto pkt = pcap.current_info();
    const uint64_t host_ts = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(pkt.timestamp).count());

    if (pkt.dst_port == info.config.udp_port_lidar) {
      if (payload_size < pf.lidar_packet_size) {
        RCLCPP_WARN(logger, "short lidar packet (%zu bytes), skipping", payload_size);
      } else {
        std::memcpy(state.lidar_packet.buf.data(), pcap.current_data(), pf.lidar_packet_size);
        state.lidar_packet.host_timestamp = host_ts;
        if (state.batcher(state.lidar_packet, state.scan)) {
          if (state.is_first_scan) {
            state.is_first_scan = false;
          } else {
            const uint64_t scan_ts = extract_scan_timestamp(state.scan, host_ts);
            state.apply_rgb_tonemap();
            on_scan(state.scan, scan_ts);
          }
        }
      }
    } else if (pkt.dst_port == info.config.udp_port_imu) {
      if (payload_size < pf.imu_packet_size) {
        RCLCPP_WARN(logger, "short imu packet (%zu bytes), skipping", payload_size);
      } else {
        std::memcpy(state.imu_packet.buf.data(), pcap.current_data(), pf.imu_packet_size);
        state.imu_packet.host_timestamp = host_ts;
        on_imu(state.imu_packet);
      }
    }
    payload_size = pcap.next_packet();
  }
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("pcap_to_mcap");

  for (const auto& p : args.pcaps) {
    if (!std::filesystem::is_regular_file(p)) {
      RCLCPP_ERROR(logger, "pcap not found: %s", p.c_str());
      return 1;
    }
  }
  if (!std::filesystem::is_regular_file(args.metadata)) {
    RCLCPP_ERROR(logger, "metadata not found: %s", args.metadata.c_str());
    return 1;
  }
  if (std::filesystem::exists(args.output_bag)) {
    RCLCPP_ERROR(logger, "output bag already exists: %s", args.output_bag.c_str());
    return 1;
  }

  const auto info = load_metadata(args.metadata);
  const auto& pf = ouster::sdk::core::get_format(info);

  const std::string frame_id = args.robot_namespace + "/os_sensor";
  const std::string lidar_topic = "/" + args.robot_namespace + "/ouster/raw_points/highres";
  const std::string imu_topic = "/" + args.robot_namespace + "/ouster/imu";

  // Writer
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // Write a temp mcap storage config selecting Zstd Slow profile.
  // The "zstd_fast" preset uses compressionLevel=Fast; for offline pcap conversion
  // we want smaller bags at the cost of more CPU during write -> compressionLevel=Slow.
  const std::string storage_config_path =
      (std::filesystem::temp_directory_path() /
       ("pcap_to_mcap_storage_" + std::to_string(getpid()) + ".yaml"))
          .string();
  {
    std::ofstream ofs(storage_config_path);
    ofs << "output:\n"
        << "  compression: Zstd\n"
        << "  compressionLevel: Slow\n";
  }

  rosbag2_storage::StorageOptions write_opts;
  write_opts.uri = args.output_bag;
  write_opts.storage_id = "mcap";
  write_opts.storage_config_uri = storage_config_path;
  write_opts.max_bagfile_size = MAX_BAGFILE_SIZE_BYTES;

  rosbag2_transport::RecordOptions record_opts{};
  auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_opts);
  writer->open(write_opts, converter_options);

  rosbag2_storage::TopicMetadata lidar_meta;
  lidar_meta.name = lidar_topic;
  lidar_meta.type = "sensor_msgs/msg/PointCloud2";
  lidar_meta.serialization_format = "cdr";
  writer->create_topic(lidar_meta);

  rosbag2_storage::TopicMetadata imu_meta;
  imu_meta.name = imu_topic;
  imu_meta.type = "sensor_msgs/msg/Imu";
  imu_meta.serialization_format = "cdr";
  writer->create_topic(imu_meta);

  uint64_t scan_counter = 0;
  uint64_t imu_counter = 0;

  // Bag log timestamp = pcap host receive time (when the packet would have arrived live),
  // not the message header timestamp (which is the sensor data timestamp from PTP).
  // For PTP-synced sensors and offline replay these can be far apart.
  uint64_t current_scan_log_ts = 0;

  auto point_cloud_processor = ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
      args.point_type, info, frame_id,
      /*apply_lidar_to_sensor_transform=*/true, args.organized, args.destagger,
      static_cast<uint32_t>(args.min_range * 1e3),
      static_cast<uint32_t>(args.max_range * 1e3), args.v_reduction, args.mask_path,
      [&](ouster_ros::PointCloudProcessor_OutputType msgs) {
        for (const auto& cloud : msgs) {
          writer->write(serialize(*cloud, lidar_topic, current_scan_log_ts));
        }
      });

  auto imu_handler = ouster_ros::ImuPacketHandler::create(info, frame_id, args.timestamp_mode,
                                                          args.ptp_utc_tai_offset);

  ScanSink on_scan = [&](const ouster::sdk::core::LidarScan& scan, uint64_t scan_ts) {
    current_scan_log_ts = scan_ts;
    point_cloud_processor(scan, scan_ts, rclcpp::Time(scan_ts));
    ++scan_counter;
  };
  ImuSink on_imu = [&](const ouster::sdk::core::ImuPacket& packet) {
    for (const auto& imu : imu_handler(packet)) {
      writer->write(serialize(imu, imu_topic, packet.host_timestamp));
      ++imu_counter;
    }
  };

  BatchState state(info, pf);
  const auto t_start = std::chrono::steady_clock::now();
  for (const auto& pcap_path : args.pcaps) {
    process_pcap(pcap_path, info, pf, state, on_scan, on_imu, logger);
  }

  writer.reset();

  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_INFO(logger, "Wrote %lu scans, %lu imu msgs in %.2fs -> %s", scan_counter, imu_counter,
              elapsed, args.output_bag.c_str());

  rclcpp::shutdown();
  return 0;
}

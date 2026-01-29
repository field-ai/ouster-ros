#include "offline_packet_converter_node.h"

#include "point_cloud_processor_factory.h"

#include <ouster/lidar_scan.h>
#include <ouster_sensor_msgs/msg/packet_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

OfflinePacketConverterNode::OfflinePacketConverterNode(const rclcpp::NodeOptions& options)
  : Node("offline_packet_converter_node", options) {
  // setup ros params
  setupParameters();

  // Validate inputs
  if (!validateInputs(input_bag_dir_, robot_name_, ouster_metadata_filepath_)) {
    throw std::runtime_error("Invalid inputs to OfflinePacketConverterNode.");
  }

  // Load metadata
  ouster_metadata_ = loadOusterMetadata(ouster_metadata_filepath_);

  // Setup topics and paths
  input_lidar_topic_ = "/" + robot_name_ + "/ouster/lidar_packets";
  frame_id_ = robot_name_ + "/os_sensor";
  output_lidar_topic_ = "/" + robot_name_ + "/raw_velodyne_points";
  output_bag_dir_ = getOutputBagDir(input_bag_dir_);
  timestamp_mode_ = "TIME_FROM_PTP_1588";

  RCLCPP_INFO(this->get_logger(),
              "Initialized with input bag: %s, output bag: %s, lidar topic: %s, frame id: %s",
              input_bag_dir_.c_str(),
              output_bag_dir_.c_str(),
              input_lidar_topic_.c_str(),
              frame_id_.c_str());

  // Validate bag
  if (!validateInputBag(input_bag_dir_)) {
    throw std::runtime_error("Unsupported input bag format.");
  }
}

void OfflinePacketConverterNode::setupParameters() {
  // Declare required parameters
  this->declare_parameter<std::string>("input_bag_dir", "");
  this->declare_parameter<std::string>("ouster_metadata_filepath", "");
  this->declare_parameter<std::string>("robot_namespace", "");

  // Declare point cloud processor parameters (from fieldai_params.yaml)
  this->declare_parameter<std::string>("point_type", "original");
  this->declare_parameter<bool>("organized", true);
  this->declare_parameter<bool>("destagger", true);
  this->declare_parameter<double>("min_range", 0.0);
  this->declare_parameter<double>("max_range", 1000.0);
  this->declare_parameter<std::string>("mask_path", "");
  this->declare_parameter<int>("v_reduction", 1);

  // Get parameter values
  input_bag_dir_ = this->get_parameter("input_bag_dir").as_string();
  ouster_metadata_filepath_ = this->get_parameter("ouster_metadata_filepath").as_string();
  robot_name_ = this->get_parameter("robot_namespace").as_string();

  point_type_ = this->get_parameter("point_type").as_string();
  organized_ = this->get_parameter("organized").as_bool();
  destagger_ = this->get_parameter("destagger").as_bool();
  min_range_mm_ = this->get_parameter("min_range").as_double() * 1e3; // it is in meters, convert to mm
  max_range_mm_ = this->get_parameter("max_range").as_double() * 1e3; // it is in meters, convert to mm
  mask_path_ = this->get_parameter("mask_path").as_string();
  rows_step_ = this->get_parameter("v_reduction").as_int();
  apply_lidar_to_sensor_transform_ = true;

  // printing to log.
  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  Input bag: %s", input_bag_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Metadata: %s", ouster_metadata_filepath_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Robot: %s", robot_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  point_type: %s", point_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  organized: %d, destagger: %d", organized_, destagger_);
  RCLCPP_INFO(this->get_logger(), "  range: [%f, %f] mm", min_range_mm_, max_range_mm_);
  RCLCPP_INFO(this->get_logger(), "  v_reduction: %d", rows_step_);
}

bool OfflinePacketConverterNode::validateInputs(const std::string& input_bag_dir,
                                                const std::string& robot_name,
                                                const std::string& ouster_metadata_filepath) {
  if (input_bag_dir.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input bag directory cannot be empty.");
    return false;
  }
  if (robot_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Robot name cannot be empty.");
    return false;
  }
  if (ouster_metadata_filepath.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Ouster metadata filepath cannot be empty.");
    return false;
  }
  return true;
}

void OfflinePacketConverterNode::convert() {
  std::string input_storage_id = "mcap";
  std::string output_storage_id = input_storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  std::string output_bag_file = output_bag_dir_;

  RCLCPP_INFO(this->get_logger(),
              "Processing bag dir: %s -> %s (splitting output at 500MB)",
              input_bag_dir_.c_str(),
              output_bag_file.c_str());

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  rosbag2_storage::StorageOptions write_storage_options;
  write_storage_options.uri = output_bag_file;
  write_storage_options.storage_id = output_storage_id;

  write_storage_options.max_bagfile_size = MAX_BAGFILE_SIZE_BYTES;
  write_storage_options.max_cache_size = MAX_CACHE_SIZE_BYTES;

  writer_->open(write_storage_options, converter_options);

  rosbag2_storage::TopicMetadata cloud_topic;
  cloud_topic.name = output_lidar_topic_;
  cloud_topic.type = "sensor_msgs/msg/PointCloud2";
  cloud_topic.serialization_format = "cdr";
  writer_->create_topic(cloud_topic);

  scan_counter_ = 0;

  processBag(input_bag_dir_, input_storage_id, converter_options);

  writer_.reset();

  RCLCPP_INFO(this->get_logger(), "Converted scans %d", scan_counter_);

  if (!rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Conversion interrupted by user.");
  } else {
    RCLCPP_INFO(this->get_logger(), "All conversions complete!");
  }
}

ouster::sensor::sensor_info OfflinePacketConverterNode::loadOusterMetadata(const std::string& metadata_file) {
  std::ifstream ifs(metadata_file);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open metadata file: %s", metadata_file.c_str());
    throw std::runtime_error("Cannot open metadata file: " + metadata_file);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  try {
    return ouster::sensor::parse_metadata(buffer.str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse Ouster metadata: %s", e.what());
    throw;
  }
}

bool OfflinePacketConverterNode::validateInputBag(const std::string& bag_dir) {
  std::filesystem::path bag(bag_dir);

  if (std::filesystem::is_directory(bag)) {
    auto metadata = bag / "metadata.yaml";
    if (!std::filesystem::exists(metadata)) {
      RCLCPP_ERROR(this->get_logger(), "Bag directory missing metadata.yaml: %s", bag.string().c_str());
      return false;
    }

    // confirm at least one .mcap exists
    for (const auto& entry : std::filesystem::directory_iterator(bag)) {
      if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
        return true;
      }
    }
    RCLCPP_ERROR(this->get_logger(), "No .mcap files found in bag directory: %s", bag.string().c_str());
    return false;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Bag path is not a directory: %s", bag_dir.c_str());
    return false;
  }
}

void OfflinePacketConverterNode::processBag(const std::string& bag_dir,
                                            const std::string& storage_id,
                                            const rosbag2_cpp::ConverterOptions& converter_options) {
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_dir;
  storage_options.storage_id = storage_id;

  reader.open(storage_options, converter_options);
  ouster::ScanBatcher batcher(ouster_metadata_);

  ouster::LidarScan scan(ouster_metadata_.format.columns_per_frame,
                         ouster_metadata_.format.pixels_per_column,
                         ouster_metadata_.format.udp_profile_lidar);

  // Use member variables
  auto point_cloud_processor = ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
      point_type_,
      ouster_metadata_,
      frame_id_,
      apply_lidar_to_sensor_transform_, // apply_lidar_to_sensor_transform
      organized_,
      destagger_,
      min_range_mm_,
      max_range_mm_,
      rows_step_,
      mask_path_,
      [this](ouster_ros::PointCloudProcessor_OutputType msgs) { this->writePointClouds(msgs); });

  bool is_first_scan = true;
  while (reader.has_next() && rclcpp::ok()) {
    auto bag_message = reader.read_next();
    if (bag_message->topic_name == input_lidar_topic_) {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      ouster_sensor_msgs::msg::PacketMsg packet_msg;

      rclcpp::Serialization<ouster_sensor_msgs::msg::PacketMsg> serialization;
      serialization.deserialize_message(&serialized_msg, &packet_msg);

      ouster::sensor::LidarPacket lidar_packet(packet_msg.buf.size());
      memcpy(lidar_packet.buf.data(), packet_msg.buf.data(), packet_msg.buf.size());
      lidar_packet.host_timestamp = static_cast<uint64_t>(bag_message->recv_timestamp);

      if (batcher(lidar_packet, scan)) {
        uint64_t scan_ts;
        auto ts_v = scan.timestamp();
        if (timestamp_mode_ == "TIME_FROM_PTP_1588") {
          auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(), [](uint64_t h) { return h != 0; });
          if (idx != ts_v.data() + ts_v.size()) {
            scan_ts = static_cast<uint64_t>(*idx);
          } else {
            scan_ts = lidar_packet.host_timestamp;
          }
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Unsupported timestamp mode, only TIME_FROM_PTP_1588 is supported, got %s",
                       timestamp_mode_.c_str());
          throw std::runtime_error("Unsupported timestamp mode, only TIME_FROM_PTP_1588 is supported, got " +
                                   timestamp_mode_);
        }
        if (is_first_scan) {
          is_first_scan = false;
          // The first scan might be partial, so we skip it to avoid issues.
          continue;
        }
        rclcpp::Time scan_msg_ts(scan_ts);
        point_cloud_processor(scan, scan_ts, scan_msg_ts);
      }
    }
  }
  reader.close();
}

void OfflinePacketConverterNode::writePointClouds(ouster_ros::PointCloudProcessor_OutputType& msgs) {
  for (auto& cloud_msg : msgs) {
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(cloud_msg.get(), serialized.get());

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = output_lidar_topic_;
    bag_msg->serialized_data =
        std::shared_ptr<rcutils_uint8_array_t>(serialized, &serialized->get_rcl_serialized_message());
    bag_msg->recv_timestamp = cloud_msg->header.stamp.sec * NANOSECONDS_PER_SECOND + cloud_msg->header.stamp.nanosec;
    writer_->write(bag_msg);
    scan_counter_++;
  }
}

std::string OfflinePacketConverterNode::getOutputBagDir(const std::string& input_bag_dir) {
  std::filesystem::path input_dir(input_bag_dir);
  std::filesystem::path parent_dir = input_dir.parent_path();
  if (parent_dir.empty()) {
    parent_dir = std::filesystem::current_path();
  }
  std::string input_dir_name = input_dir.filename().string();
  std::string output_dir_name = replaceRawWithPointcloud(input_dir_name);
  std::filesystem::path output_dir = parent_dir / output_dir_name;
  if (std::filesystem::exists(output_dir)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Output directory %s already exists. Files may be overwritten.",
                 output_dir.string().c_str());
    throw std::runtime_error("Output directory already exists: " + output_dir.string());
  }
  return output_dir.string();
}

std::string OfflinePacketConverterNode::replaceRawWithPointcloud(const std::string& name) {
  std::string output_name = name;
  const std::string raw_suffix = "_raw_";
  const std::string pointcloud_suffix = "_pointcloud_";
  size_t pos = output_name.find(raw_suffix);
  if (pos != std::string::npos) {
    output_name.replace(pos, raw_suffix.length(), pointcloud_suffix);
  }
  return output_name;
}

OfflinePacketConverterNode::~OfflinePacketConverterNode() {
  if (writer_) {
    writer_.reset();
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OfflinePacketConverterNode>();

  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  try {
    node->convert();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  auto time_taken = std::chrono::steady_clock::now() - start_time;
  RCLCPP_INFO(node->get_logger(), "Total time taken: %.2f seconds", std::chrono::duration<double>(time_taken).count());

  rclcpp::shutdown();
  return 0;
}
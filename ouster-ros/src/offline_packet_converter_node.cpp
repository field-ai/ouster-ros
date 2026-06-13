#include "offline_packet_converter_node.h"

OfflinePacketConverterNode::OfflinePacketConverterNode(const rclcpp::NodeOptions& options)
  : Node("offline_packet_converter_node", options) {
  // setup ros params
  setupParameters();

  // initialize paths and validate
  if (!init()) {
    throw std::runtime_error("OfflinePacketConverterNode initialization failed");
  }

  RCLCPP_INFO(this->get_logger(),
              "Initialized with input bag: %s, output bag: %s, lidar topic: %s, frame id: %s",
              input_bag_dir_.c_str(),
              output_bag_dir_.c_str(),
              input_lidar_topic_.c_str(),
              frame_id_.c_str());
}

bool OfflinePacketConverterNode::isDirectory(const std::filesystem::path& dir_path) {
  if (!std::filesystem::exists(dir_path) || !std::filesystem::is_directory(dir_path)) {
    RCLCPP_ERROR(this->get_logger(), "Base directory does not exist: %s", dir_path.c_str());
    return false;
  }
  return true;
}

bool OfflinePacketConverterNode::isFile(const std::filesystem::path& file_path) {
  if (!std::filesystem::exists(file_path) || !std::filesystem::is_regular_file(file_path)) {
    RCLCPP_ERROR(this->get_logger(), "File does not exist: %s", file_path.string().c_str());
    return false;
  }
  return true;
}

bool OfflinePacketConverterNode::init() {
  // Load and validate base directory
  std::filesystem::path data_dir_fp(data_dir_);
  if (!isDirectory(data_dir_fp)) {
    return false;
  }

  // load and validate ouster metadata file
  std::filesystem::path log_dir = data_dir_fp / "log";
  std::filesystem::path metadata_path = log_dir / "ouster_metadata.json";

  if (!isFile(metadata_path)) {
    return false;
  }

  ouster_metadata_filepath_ = metadata_path.string();
  ouster_metadata_ = loadOusterMetadata(ouster_metadata_filepath_);

  // Load and validate rosbag2 directory
  std::filesystem::path rosbag2_dir = data_dir_fp / "rosbag2";

  if (!isDirectory(rosbag2_dir)) {
    return false;
  }

  auto lidar_bags = findDirsByRegex(rosbag2_dir.string(), LIDAR_BAG_PATTERN);

  if (lidar_bags.empty()) {
    return false;
  }

  if (lidar_bags.size() > 1) {
    RCLCPP_WARN(this->get_logger(),
                "Found %zu lidar bag directories, using first one: %s",
                lidar_bags.size(),
                lidar_bags.begin()->second.c_str());
  }

  // choose the first bag dir found
  input_bag_dir_ = lidar_bags.begin()->second;

  // Validate bag dir
  if (!validateInputBagDir(input_bag_dir_)) {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Using input bag directory: %s", input_bag_dir_.c_str());

  // Setup output bag dir
  std::string output_dir_name = replaceRawWithPointcloud(std::filesystem::path(input_bag_dir_).filename().string());
  output_bag_dir_ = rosbag2_dir / output_dir_name;

  // Setup topics and paths
  input_lidar_topic_ = "/" + robot_name_ + "/ouster/lidar_packets";
  input_imu_topic_ = "/" + robot_name_ + "/ouster/imu";
  frame_id_ = robot_name_ + "/os_sensor";
  output_lidar_topic_ = "/" + robot_name_ + "/raw_velodyne_points";
  output_dual_lidar_topic_ = "/" + robot_name_ + "/dual_return/raw_velodyne_points";
  timestamp_mode_ = "TIME_FROM_PTP_1588";

  if (std::filesystem::exists(output_bag_dir_)) {
    RCLCPP_ERROR(
        this->get_logger(), "Output directory %s already exists. Files may be overwritten.", output_bag_dir_.c_str());
    return false;
  }

  // Setup reader and writer
  if (!setupReaderWriter()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to setup reader and writer.");
    return false;
  }
  return true;
}

bool OfflinePacketConverterNode::setupReaderWriter() {
  const std::string storage_id = "mcap";
  // Common converter options
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // Setup reader
  rosbag2_storage::StorageOptions read_storage_options;
  read_storage_options.uri = input_bag_dir_;
  read_storage_options.storage_id = storage_id;

  reader_.reset();
  reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(read_storage_options);
  reader_->open(read_storage_options, converter_options);

  // validate input bag has expected topics
  bool bag_has_input_lidar_topic = false;
  bool bag_has_input_imu_topic = false;
  auto topics = reader_->get_all_topics_and_types();
  for (const auto& topic : topics) {
    if (topic.name == input_lidar_topic_) {
      bag_has_input_lidar_topic = true;
    } else if (topic.name == input_imu_topic_) {
      bag_has_input_imu_topic = true;
    }
  }
  if (!bag_has_input_lidar_topic) {
    RCLCPP_ERROR(this->get_logger(), "Input bag does not contain expected lidar topic: %s", input_lidar_topic_.c_str());
    return false;
  }
  if (!bag_has_input_imu_topic) {
    RCLCPP_WARN(this->get_logger(), "Input bag does not contain IMU topic: %s. IMU data will not be included in output.",
                input_imu_topic_.c_str());
  }

  // // Setup writer with compression
  rosbag2_transport::RecordOptions record_options{};

  rosbag2_storage::StorageOptions write_storage_options;
  write_storage_options.uri = output_bag_dir_;
  write_storage_options.storage_id = storage_id;
  write_storage_options.storage_preset_profile = "zstd_fast";
  write_storage_options.max_bagfile_size = MAX_BAGFILE_SIZE_BYTES;
  writer_.reset();
  writer_ = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
  writer_->open(write_storage_options, converter_options);

  // Create output topics
  rosbag2_storage::TopicMetadata cloud_topic;
  cloud_topic.name = output_lidar_topic_;
  cloud_topic.type = "sensor_msgs/msg/PointCloud2";
  cloud_topic.serialization_format = "cdr";
  writer_->create_topic(cloud_topic);

  // Dual-return profile: also emit the second return on a parallel topic.
  if (ouster_metadata_.num_returns() > 1) {
    rosbag2_storage::TopicMetadata dual_cloud_topic;
    dual_cloud_topic.name = output_dual_lidar_topic_;
    dual_cloud_topic.type = "sensor_msgs/msg/PointCloud2";
    dual_cloud_topic.serialization_format = "cdr";
    writer_->create_topic(dual_cloud_topic);
  }

  if (bag_has_input_imu_topic) {
    rosbag2_storage::TopicMetadata imu_topic;
    imu_topic.name = input_imu_topic_;
    imu_topic.type = "sensor_msgs/msg/Imu";
    imu_topic.serialization_format = "cdr";
    writer_->create_topic(imu_topic);
  }

  return true;
}

std::map<int, std::string> OfflinePacketConverterNode::findDirsByRegex(const std::string& search_dir,
                                                                       const std::string& pattern) {
  std::map<int, std::string> result;
  std::regex dir_regex(pattern);

  for (const auto& entry : std::filesystem::directory_iterator(search_dir)) {
    if (!entry.is_directory()) {
      continue;
    }
    std::string dir_name = entry.path().filename().string();
    std::smatch match;

    if (std::regex_match(dir_name, match, dir_regex)) {
      int idx = static_cast<int>(result.size()); // Sequential index
      result.try_emplace(idx, entry.path().string());
    }
  }

  return result;
}

void OfflinePacketConverterNode::setupParameters() {
  // Declare required parameters
  this->declare_parameter<std::string>("data_dir", "");
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
  data_dir_ = this->get_parameter("data_dir").as_string();
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
  RCLCPP_INFO(this->get_logger(), "  Data dir: %s", data_dir_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Robot: %s", robot_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  point_type: %s", point_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "  organized: %d, destagger: %d", organized_, destagger_);
  RCLCPP_INFO(this->get_logger(), "  range: [%f, %f] mm", min_range_mm_, max_range_mm_);
  RCLCPP_INFO(this->get_logger(), "  v_reduction: %d", rows_step_);
}

void OfflinePacketConverterNode::convert() {
  if (!process()) {
    throw std::runtime_error("Failed to process bag.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Conversion process complete. Total scans processed: %d", scan_counter_);

  if (!rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Conversion interrupted by user.");
  } else {
    RCLCPP_INFO(this->get_logger(), "All conversions complete!");
  }
}

ouster::sdk::core::SensorInfo OfflinePacketConverterNode::loadOusterMetadata(const std::string& metadata_file) {
  std::ifstream ifs(metadata_file);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open metadata file: %s", metadata_file.c_str());
    throw std::runtime_error("Cannot open metadata file: " + metadata_file);
  }
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  try {
    return ouster::sdk::core::SensorInfo(buffer.str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse Ouster metadata: %s", e.what());
    throw;
  }
}

bool OfflinePacketConverterNode::validateInputBagDir(const std::string& bag_dir) {
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

ouster::sdk::core::LidarPacket
OfflinePacketConverterNode::deserializeLidarPacket(const rosbag2_storage::SerializedBagMessage& bag_message) {
  rclcpp::SerializedMessage serialized_msg(*bag_message.serialized_data);
  ouster_sensor_msgs::msg::PacketMsg packet_msg;

  rclcpp::Serialization<ouster_sensor_msgs::msg::PacketMsg> serialization;
  serialization.deserialize_message(&serialized_msg, &packet_msg);

  ouster::sdk::core::LidarPacket lidar_packet(packet_msg.buf.size());
  std::memcpy(lidar_packet.buf.data(), packet_msg.buf.data(), packet_msg.buf.size());
  lidar_packet.host_timestamp = static_cast<uint64_t>(bag_message.recv_timestamp);

  return lidar_packet;
}

uint64_t OfflinePacketConverterNode::extractScanTimestamp(const ouster::sdk::core::LidarScan& scan, uint64_t fallback_timestamp) {
  auto ts_v = scan.timestamp();
  auto it = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(), [](uint64_t t) { return t != 0; });

  return (it != ts_v.data() + ts_v.size()) ? *it : fallback_timestamp;
}

bool OfflinePacketConverterNode::process() {
  if (!reader_ || !writer_) {
    RCLCPP_ERROR(this->get_logger(), "Reader or Writer not initialized.");
    return false;
  }
  // setup ouster processing pipeline
  ouster::sdk::core::ScanBatcher batcher(ouster_metadata_);

  ouster::sdk::core::LidarScan scan(ouster_metadata_.format.columns_per_frame,
                         ouster_metadata_.format.pixels_per_column,
                         ouster_metadata_.format.udp_profile_lidar);

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
  scan_counter_ = 0;

  while (reader_->has_next() && rclcpp::ok()) {
    auto bag_message = reader_->read_next();

    if (bag_message->topic_name == input_imu_topic_) {
      writer_->write(bag_message);
      continue;
    }

    if (bag_message->topic_name != input_lidar_topic_) {
      continue;
    }

    auto lidar_packet = deserializeLidarPacket(*bag_message);

    if (!batcher(lidar_packet, scan)) {
      // incomplete scan, skipping. We only want completed scans.
      continue;
    }

    if (is_first_scan) {
      is_first_scan = false;
      // The first scan might be partial, so we skip it to avoid issues.
      continue;
    }

    uint64_t scan_ts = extractScanTimestamp(scan, lidar_packet.host_timestamp);

    point_cloud_processor(scan, scan_ts, rclcpp::Time(scan_ts));
    scan_counter_++;
  }
  reader_.reset();
  writer_.reset();
  return true;
}

void OfflinePacketConverterNode::writePointClouds(ouster_ros::PointCloudProcessor_OutputType& msgs) {
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  // msgs[0] is the first return; index >= 1 is the dual (second) return.
  for (size_t i = 0; i < msgs.size(); ++i) {
    const auto& cloud_msg = msgs[i];
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    serialization.serialize_message(cloud_msg.get(), serialized.get());

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = (i == 0) ? output_lidar_topic_ : output_dual_lidar_topic_;
    bag_msg->serialized_data =
        std::shared_ptr<rcutils_uint8_array_t>(serialized, &serialized->get_rcl_serialized_message());
    bag_msg->recv_timestamp =
        static_cast<uint64_t>(cloud_msg->header.stamp.sec) * NANOSECONDS_PER_SECOND + cloud_msg->header.stamp.nanosec;
    writer_->write(bag_msg);
  }
}

std::string OfflinePacketConverterNode::replaceRawWithPointcloud(const std::string& name) {
  static const std::regex lidar_regex(LIDAR_BAG_PATTERN);
  return std::regex_replace(name, lidar_regex, LIDAR_POINTCLOUD_REPLACE);
}

OfflinePacketConverterNode::~OfflinePacketConverterNode() {
  if (writer_) {
    writer_.reset();
  }
  if (reader_) {
    reader_.reset();
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
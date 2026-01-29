#include "offline_packet_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ouster_sensor_msgs/msg/packet_msg.hpp>

#include <ouster/lidar_scan.h>

#include "point_cloud_processor_factory.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

OfflinePacketConverter::OfflinePacketConverter(const std::string& input_bag_dir, 
                          const std::string& ouster_metadata_file,
                          const std::string& robot_name,
                          const std::string& timestamp_mode)
    :   input_bag_dir_(input_bag_dir),
        robot_name_(robot_name),
        timestamp_mode_(timestamp_mode)
    {
    if (!validateInputs(input_bag_dir_, robot_name_, timestamp_mode_)) {
        throw std::runtime_error("Invalid inputs to OfflinePacketConverter.");
    }
    ouster_metadata_ = loadOusterMetadata(ouster_metadata_file);

    input_lidar_topic_ = "/" + robot_name_ + "/ouster/lidar_packets";
    frame_id_ = robot_name_ + "/os_sensor";
    output_lidar_topic_ = "/" + robot_name_ + "/raw_velodyne_points";
    output_bag_dir_ = getOutputBagDir(input_bag_dir_);
    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                "Initialized OfflinePacketConverter with input bag: %s, output bag: %s, lidar topic: %s, frame id: %s",
                input_bag_dir_.c_str(), output_bag_dir_.c_str(),
                input_lidar_topic_.c_str(), frame_id_.c_str());
    bool is_mcap = validateInputBag(input_bag_dir_);
    if (!is_mcap) {
        throw std::runtime_error("Unsupported input bag format.");
    }
}

bool OfflinePacketConverter::validateInputs(const std::string& input_bag_dir,
                    const std::string& robot_name,
                    const std::string& timestamp_mode) {
    
    if (input_bag_dir.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Input bag directory cannot be empty.");
        return false;
    }
    if (robot_name.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Robot name cannot be empty.");
        return false;
    }
    if (timestamp_mode != "TIME_FROM_PTP_1588")
    {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Unsupported timestamp mode: %s. Only TIME_FROM_PTP_1588 is supported.",
                     timestamp_mode.c_str());
        return false;
    }
    return true;
}

void OfflinePacketConverter::convert() {
    
    std::string input_storage_id = "mcap";
    std::string output_storage_id = input_storage_id;
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    
    std::string output_bag_file = output_bag_dir_;

    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                "Processing bag dir: %s -> %s (splitting output at 500MB)",
                input_bag_dir_.c_str(), output_bag_file.c_str());

    
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

    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "Converted scans %d", scan_counter_);
    
    if (!rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "Conversion interrupted by user.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "All conversions complete!");
    }
}

ouster::sensor::sensor_info OfflinePacketConverter::loadOusterMetadata(const std::string& metadata_file) {
    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Cannot open metadata file: %s", metadata_file.c_str());
        throw std::runtime_error("Cannot open metadata file: " + metadata_file);
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    try
    {
        return ouster::sensor::parse_metadata(buffer.str());
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Failed to parse Ouster metadata: %s", e.what());
        throw;
    }
}

bool OfflinePacketConverter::validateInputBag(const std::string& bag_dir) {
  std::filesystem::path bag(bag_dir);

  if (std::filesystem::is_directory(bag)) {
    auto metadata = bag / "metadata.yaml";
    if (!std::filesystem::exists(metadata)) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Bag directory missing metadata.yaml: %s", bag.string().c_str());
        return false;
    }

    // confirm at least one .mcap exists
    for (const auto& entry : std::filesystem::directory_iterator(bag)) {
      if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
        return true;
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                 "No .mcap files found in bag directory: %s", bag.string().c_str());
    return false;
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                 "Bag path is not a directory: %s", bag_dir.c_str());
    return false;
  }
}

void OfflinePacketConverter::processBag(const std::string& bag_file, 
                           const std::string& storage_id,
                           const rosbag2_cpp::ConverterOptions& converter_options) {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = storage_id;
    
    reader.open(storage_options, converter_options);
    ouster::ScanBatcher batcher(ouster_metadata_);
    
    ouster::LidarScan scan(
        ouster_metadata_.format.columns_per_frame,
        ouster_metadata_.format.pixels_per_column,
        ouster_metadata_.format.udp_profile_lidar
    );
    std::string point_type = "original"; // original = ouster_ros::Point
    bool apply_lidar_to_sensor_transform = false;
    bool organized = true;
    bool destagger = true;
    int min_range_m = 0;
    int max_range_m = 10000;
    int rows_step = 1;
    std::string mask_path = "";  
    auto point_cloud_processor = ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
        point_type,
        ouster_metadata_,                   // sensor_info
        frame_id_,                          // frame_id
        apply_lidar_to_sensor_transform,    // apply_lidar_to_sensor_transform
        organized,                          // organized (512x128)
        destagger,                          // destagger (keep as-is, no destagger)
        min_range_m,                        // min_range (m)
        max_range_m,                        // max_range (m) (using default value)
        rows_step,                          // rows_step (use all rows)
        mask_path,                          // mask_path (no mask)
        [this](ouster_ros::PointCloudProcessor_OutputType msgs) {
            this->writePointClouds(msgs);
        }
    );
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
                    auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                            [](uint64_t h) { return h != 0; });
                    if (idx != ts_v.data() + ts_v.size()) {
                        scan_ts = static_cast<uint64_t>(*idx);
                    } else {
                        scan_ts = lidar_packet.host_timestamp;
                    }
                }
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                                 "Unsupported timestamp mode, only TIME_FROM_PTP_1588 is supported, got %s", timestamp_mode_.c_str());
                    throw std::runtime_error("Unsupported timestamp mode, only TIME_FROM_PTP_1588 is supported, got " + timestamp_mode_);
                }
                if (is_first_scan) {
                    is_first_scan = false;
                    continue;  // skip first scan to avoid partial scans
                }
                rclcpp::Time scan_msg_ts(scan_ts);
                point_cloud_processor(
                    scan,                           // The complete scan
                    scan_ts,                        // Timestamp in nanoseconds
                    scan_msg_ts                     // ROS time
                );
            }
        }
    }
}

void OfflinePacketConverter::writePointClouds(ouster_ros::PointCloudProcessor_OutputType& msgs) {
    for (auto& cloud_msg : msgs) {
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        auto serialized = std::make_shared<rclcpp::SerializedMessage>();
        serialization.serialize_message(cloud_msg.get(), serialized.get());
        
        auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_msg->topic_name = output_lidar_topic_;
        bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            &serialized->get_rcl_serialized_message(),
            [serialized](rcutils_uint8_array_t*) {});
        bag_msg->recv_timestamp = cloud_msg->header.stamp.nanosec + 
                                 cloud_msg->header.stamp.sec * NANOSECONDS_PER_SECOND;
        
        writer_->write(bag_msg);
        scan_counter_++;
    }
}

std::string OfflinePacketConverter::getOutputBagDir(const std::string& input_bag_dir) {
    std::filesystem::path input_dir(input_bag_dir);
    std::filesystem::path parent_dir = input_dir.parent_path();
    if (parent_dir.empty()) {
        parent_dir = std::filesystem::current_path();
    }
    std::string input_dir_name = input_dir.filename().string();
    std::string output_dir_name = replaceRawWithPointcloud(input_dir_name);
    std::filesystem::path output_dir = parent_dir / output_dir_name;
    if (std::filesystem::exists(output_dir)) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                    "Output directory %s already exists. Files may be overwritten.",
                    output_dir.string().c_str());
        throw std::runtime_error("Output directory already exists: " + output_dir.string());
    }
    return output_dir.string();
}

std::string OfflinePacketConverter::replaceRawWithPointcloud(const std::string& name) {
    std::string output_name = name;
    const std::string raw_suffix = "_raw_";
    const std::string pointcloud_suffix = "_pointcloud_";
    size_t pos = output_name.find(raw_suffix);
    if (pos != std::string::npos) {
        output_name.replace(pos, raw_suffix.length(), pointcloud_suffix);
    }
    return output_name;
}

OfflinePacketConverter::~OfflinePacketConverter(){
    if (writer_) {
        writer_.reset();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <input_bag_dir> <ouster_metadata_json> <robot_name>" 
                  << std::endl;
        return 1;
    }
    
    std::string input_bag_dir = argv[1];
    std::string ouster_metadata_file = argv[2];
    std::string robot_name = argv[3];
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    try {
        OfflinePacketConverter converter(input_bag_dir, ouster_metadata_file, robot_name);
        converter.convert();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    auto time_taken = std::chrono::steady_clock::now() - start_time;
    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                "Total time taken: %.2f seconds",
                std::chrono::duration<double>(time_taken).count());
    
    rclcpp::shutdown();
    return 0;
}
#include "offline_packet_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>
#include <chrono>

// ADD THESE:
#include <ouster/lidar_scan.h>
#include <ouster/impl/cartesian.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ouster_ros/os_point.h"
#include <pcl_conversions/pcl_conversions.h>
#include "point_cloud_compose.h"
#include "point_meta_helpers.h"

OfflinePacketConverter::OfflinePacketConverter(const std::string& input_bag_dir, 
                          const std::string& ouster_metadata_file,
                          const std::string& robot_name,
                          const std::string& timestamp_mode)
    :   input_bag_dir_(input_bag_dir),
        robot_name_(robot_name),
        timestamp_mode_(timestamp_mode),
        input_rosbags_({}) {

    ouster_metadata_ = loadOusterMetadata(ouster_metadata_file);

    input_lidar_topic_ = "/" + robot_name_ + "/ouster/lidar_packets";
    frame_id_ = robot_name_ + "/os_sensor";
    output_lidar_topic_ = "/" + robot_name_ + "/raw_velodyne_points";
    output_bag_dir_ = getOutputBagDir(input_bag_dir_);

    if (!getBagsFromDir(input_bag_dir_, input_rosbags_)) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "No bag files found in input directory: %s", input_bag_dir_.c_str());
        throw std::runtime_error("No bag files found in: " + input_bag_dir_);
    }
    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                "Initialized OfflinePacketConverter with input bag: %s, output bag: %s, lidar topic: %s, frame id: %s",
                input_bag_dir_.c_str(), output_bag_dir_.c_str(),
                input_lidar_topic_.c_str(), frame_id_.c_str());
}

void OfflinePacketConverter::convert() {
    
    // Check only storage format from first bag, assume all bags are same format
    bool is_mcap = isMcapBag(input_rosbags_[0]);
    if (!is_mcap) {
        RCLCPP_ERROR(rclcpp::get_logger("OfflinePacketConverter"),
                     "Only MCAP bag format is supported.");
        throw std::runtime_error("Unsupported input bag format.");
    }
    std::string input_storage_id = "mcap";
    std::string output_storage_id = input_storage_id;
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                "Starting conversion of %zu bag(s)...", input_rosbags_.size());
    
    // Process each bag file - create separate output for each
    for (size_t bag_idx = 0; bag_idx < input_rosbags_.size(); ++bag_idx) {
        const auto& bag_file = input_rosbags_[bag_idx];
        
        if (!rclcpp::ok()) {
            RCLCPP_WARN(rclcpp::get_logger("OfflinePacketConverter"), "Conversion interrupted by user.");
            break;
        }
        
        std::filesystem::path input_bag_path(bag_file);
        std::filesystem::path input_filename = input_bag_path.filename();
        std::string output_bag_file = output_bag_dir_ + "/" + getOutputBagFilename(input_filename.string());

        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                    "Processing bag %zu/%zu: %s -> %s",
                    bag_idx + 1, input_rosbags_.size(),
                    bag_file.c_str(), output_bag_file.c_str());
        
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        rosbag2_storage::StorageOptions write_storage_options;
        write_storage_options.uri = output_bag_file;
        write_storage_options.storage_id = output_storage_id;
        
        writer_->open(write_storage_options, converter_options);

        // Create topic for point cloud
        rosbag2_storage::TopicMetadata cloud_topic;
        cloud_topic.name = output_lidar_topic_;
        cloud_topic.type = "sensor_msgs/msg/PointCloud2";
        cloud_topic.serialization_format = "cdr";
        writer_->create_topic(cloud_topic);
        
        scan_counter_ = 0;
        
        // Process this bag
        processSingleBag(bag_file, input_storage_id, converter_options);
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "Converted scans %d", scan_counter_);
        
        writer_.reset();
    }
    if (!rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "Conversion interrupted by user.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"), "All conversions complete!");
    }
}

ouster::sensor::sensor_info OfflinePacketConverter::loadOusterMetadata(const std::string& metadata_file) {
    std::ifstream ifs(metadata_file);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open metadata file: " + metadata_file);
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ouster::sensor::sensor_info ouster_metadata = ouster::sensor::parse_metadata(buffer.str());
    return ouster_metadata;
}

bool OfflinePacketConverter::getBagsFromDir(const std::string& bag_dir, std::vector<std::string>& bags) {
    std::filesystem::path p(bag_dir);

    if (std::filesystem::is_directory(bag_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(p)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                if (ext == ".mcap") {
                    bags.push_back(entry.path().string());
                }
            }
        }
        std::sort(bags.begin(), bags.end());
    }
    return !bags.empty();
}

bool OfflinePacketConverter::isMcapBag(const std::string& bag_path) {
    std::filesystem::path bag(bag_path);
    
    if (std::filesystem::is_regular_file(bag_path)) {
        std::string ext = bag.extension().string();
        if (ext == ".mcap") {
            return true;
        }
        else{
            throw std::runtime_error("Cannot detect bag format for: " + bag_path);
        }
    }
    throw std::runtime_error("Bag path is not a file: " + bag_path);
}

void OfflinePacketConverter::processSingleBag(const std::string& bag_file, 
                           const std::string& storage_id,
                           const rosbag2_cpp::ConverterOptions& converter_options) {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = storage_id;
    
    reader.open(storage_options, converter_options);
    
    // Create ScanBatcher - THIS is the key!
    ouster::ScanBatcher batcher(ouster_metadata_);
    
    // Create LidarScan to accumulate data
    ouster::LidarScan scan(
        ouster_metadata_.format.columns_per_frame,
        ouster_metadata_.format.pixels_per_column,
        ouster_metadata_.format.udp_profile_lidar
    );
    
    while (reader.has_next() && rclcpp::ok()) {
        auto bag_message = reader.read_next();
        
        if (bag_message->topic_name == input_lidar_topic_) {
            // Deserialize packet
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            ouster_sensor_msgs::msg::PacketMsg packet_msg;
            
            rclcpp::Serialization<ouster_sensor_msgs::msg::PacketMsg> serialization;
            serialization.deserialize_message(&serialized_msg, &packet_msg);
            
            // Convert to Ouster LidarPacket
            ouster::sensor::LidarPacket lidar_packet(packet_msg.buf.size());
            memcpy(lidar_packet.buf.data(), packet_msg.buf.data(), packet_msg.buf.size());
            lidar_packet.host_timestamp = static_cast<uint64_t>(bag_message->recv_timestamp);
            
            current_timestamp_ = bag_message->recv_timestamp;
            
            bool scan_complete = batcher(lidar_packet, scan);
            
            if (scan_complete) {
                processCompleteScan(scan);                
            }
        }
    }
}

void OfflinePacketConverter::processCompleteScan(const ouster::LidarScan& scan) {
    static ouster::XYZLut lut = ouster::make_xyz_lut(ouster_metadata_);
    
    auto points_double = ouster::cartesian(scan, lut);
    ouster::PointsF points = points_double.cast<float>();
    
    size_t h = ouster_metadata_.format.pixels_per_column;
    size_t w = ouster_metadata_.format.columns_per_frame;
    
    pcl::PointCloud<ouster_ros::Point> pcl_cloud;
    pcl_cloud.header.frame_id = frame_id_;
    pcl_cloud.header.stamp = current_timestamp_ / 1000;
    
    pcl_cloud.width = w;
    pcl_cloud.height = h;
    pcl_cloud.is_dense = false;
    pcl_cloud.points.resize(w * h);
    
    auto range = scan.field<uint32_t>(ouster::sensor::ChanField::RANGE);
    auto signal = scan.field<uint16_t>(ouster::sensor::ChanField::SIGNAL);
    auto reflectivity = scan.field<uint16_t>(ouster::sensor::ChanField::REFLECTIVITY);
    auto near_ir = scan.field<uint16_t>(ouster::sensor::ChanField::NEAR_IR);
    auto timestamps = scan.timestamp();

    for (size_t u = 0; u < w; u++) {
        for (size_t v = 0; v < h; v++) {
            size_t xyz_idx = u * h + v;
            size_t cloud_idx = v * w + u;
            
            ouster_ros::Point& point = pcl_cloud.points[cloud_idx];
            
            point.x = points(xyz_idx, 0);
            point.y = points(xyz_idx, 1);
            point.z = points(xyz_idx, 2);
            
            point.intensity = static_cast<float>(signal(v, u));

            point.t = static_cast<uint32_t>(timestamps[u]);
            point.reflectivity = reflectivity(v, u);
            point.ring = static_cast<uint16_t>(v);
            point.ambient = near_ir(v, u);
            point.range = range(v, u);
            point.column = static_cast<uint16_t>(u);
        }
    }
    
    // Debug for first few scans
    if (scan_counter_ < 3) {
        size_t valid = 0, invalid = 0;
        for (size_t i = 0; i < pcl_cloud.points.size(); i++) {
            if (pcl_cloud.points[i].range == 0) invalid++;
            else valid++;
        }
        RCLCPP_INFO(rclcpp::get_logger("OfflinePacketConverter"),
                   "Scan %d: %zu valid, %zu invalid points",
                   scan_counter_, valid, invalid);
    }
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(current_timestamp_);
    cloud_msg.header.frame_id = frame_id_;
    
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
    auto serialized_cloud = std::make_shared<rclcpp::SerializedMessage>();
    cloud_serialization.serialize_message(&cloud_msg, serialized_cloud.get());
    
    auto cloud_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    cloud_bag_msg->topic_name = output_lidar_topic_;
    cloud_bag_msg->serialized_data = 
        std::shared_ptr<rcutils_uint8_array_t>(
            &serialized_cloud->get_rcl_serialized_message(),
            [serialized_cloud](rcutils_uint8_array_t*) {});
    cloud_bag_msg->recv_timestamp = current_timestamp_;
    
    writer_->write(cloud_bag_msg);
    scan_counter_++;
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
    std::filesystem::create_directories(output_dir);
    if (!std::filesystem::exists(output_dir)) {
        throw std::runtime_error("Output directory does not exist: " + output_dir.string());
    }
    return output_dir.string();
}

std::string OfflinePacketConverter::getOutputBagFilename(const std::string& input_bag_filename) {
    std::string output_filename = replaceRawWithPointcloud(input_bag_filename);
    return output_filename;
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

OfflinePacketConverter::~OfflinePacketConverter() {
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
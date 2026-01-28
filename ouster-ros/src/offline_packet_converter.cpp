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

OfflinePacketConverter::OfflinePacketConverter(const std::string& input_bag_dir, 
                          const std::string& ouster_metadata_file,
                          const std::string& robot_name,
                          const std::string& timestamp_mode)
    :   input_bag_dir_(input_bag_dir),
        robot_name_(robot_name),
        timestamp_mode_(timestamp_mode),
        processing_complete_(false),
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
    setupProcessors();
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
    // Setup reader for this bag
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = storage_id;
    
    reader.open(storage_options, converter_options);
    
    // Process messages from this bag
    while (reader.has_next() && rclcpp::ok()) {
        auto bag_message = reader.read_next();
        
        // Only process lidar packets, ignore everything else
        if (bag_message->topic_name == input_lidar_topic_) {
            // Process lidar packets
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            ouster_sensor_msgs::msg::PacketMsg packet_msg;
            
            rclcpp::Serialization<ouster_sensor_msgs::msg::PacketMsg> serialization;
            serialization.deserialize_message(&serialized_msg, &packet_msg);
            
            ouster::sensor::LidarPacket lidar_packet(packet_msg.buf.size());
            memcpy(lidar_packet.buf.data(), packet_msg.buf.data(), packet_msg.buf.size());
            lidar_packet.host_timestamp = static_cast<uint64_t>(bag_message->recv_timestamp);
            
            current_timestamp_ = bag_message->recv_timestamp;
            
            // Process packet through handler
            if (lidar_packet_handler_) {
                lidar_packet_handler_(lidar_packet);
            }
            waitForProcessing();
        }
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

void OfflinePacketConverter::setupProcessors() {    
    std::vector<ouster_ros::LidarScanProcessor> processors;
    
    // Setup point cloud processor (same parameters as os_cloud_node)
    /*
    TODO: We probably want a better way to provide these default parameters,
    so when we change these while launching the ros2 driver, they are also applied here.
    One way is to have a config file or command line args for these parameters,
    which will be same between driver and offline converter.
    */

    bool organized = true;
    bool destagger = true;
    uint32_t min_range = 0;      // in mm
    uint32_t max_range = 1000000; // in mm (1000m)
    int v_reduction = 1;
    std::string mask_path = "";
    std::string point_type = "original";
    
    processors.push_back(
        ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
            point_type,
            ouster_metadata_, 
            frame_id_,
            false,
            organized, 
            destagger, 
            min_range, 
            max_range, 
            v_reduction, 
            mask_path,
            [this](ouster_ros::PointCloudProcessor_OutputType msgs) {
                this->handlePointClouds(msgs);
            }
        )
    );
    
    // Create lidar packet handler with smaller queue (reduce buffering)
    double ptp_utc_tai_offset = 0.0; // it's zero in the ouster.launch.py file by default.
    double min_scan_valid_columns_ratio = 0.0; // it's zero by default.
    
    lidar_packet_handler_ = ouster_ros::LidarPacketHandler::create(
        ouster_metadata_, 
        processors, 
        timestamp_mode_,
        static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
        min_scan_valid_columns_ratio
    );
}
    
void OfflinePacketConverter::waitForProcessing() {
    std::unique_lock<std::mutex> lock(processing_mutex_);
    // Give processing thread time to catch up
    processing_cv_.wait_for(lock, std::chrono::milliseconds(1));
}

void OfflinePacketConverter::handlePointClouds(ouster_ros::PointCloudProcessor_OutputType msgs) {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    
    // Process each return (usually just one for standard processing)
    for (size_t i = 0; i < msgs.size(); ++i) {
        auto& cloud_msg = msgs[i];
        
        // Serialize and write point cloud to bag
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
        auto serialized_cloud = std::make_shared<rclcpp::SerializedMessage>();
        cloud_serialization.serialize_message(cloud_msg.get(), serialized_cloud.get());
        
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
    processing_cv_.notify_all();
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
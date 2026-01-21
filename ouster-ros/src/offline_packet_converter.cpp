#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ouster_sensor_msgs/msg/packet_msg.hpp>
#include <ouster_ros/os_ros.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>

#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "point_cloud_processor_factory.h"

#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <mutex>
#include <condition_variable>

class OfflinePacketConverter {
public:
    OfflinePacketConverter(const std::string& input_bag, 
                          const std::string& output_bag,
                          const std::string& metadata_file,
                          const std::string& lidar_topic = "/lidar_packets",
                          const std::string& point_type = "original",
                          const std::string& timestamp_mode = "TIME_FROM_INTERNAL_OSC")
        : input_bag_path_(input_bag),
          output_bag_path_(output_bag),
          metadata_file_(metadata_file),
          lidar_topic_(lidar_topic),
          point_type_(point_type),
          timestamp_mode_(timestamp_mode),
          processing_complete_(false) {
        
        // Load sensor metadata
        std::ifstream ifs(metadata_file);
        if (!ifs.is_open()) {
            throw std::runtime_error("Cannot open metadata file: " + metadata_file);
        }
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        std::string metadata_str = buffer.str();
        
        info_ = ouster::sensor::parse_metadata(metadata_str);
        
        // Setup point cloud processor
        setup_processors();
    }

    void convert() {
        // Detect storage format from input bag
        std::string input_storage_id = detect_storage_format(input_bag_path_);
        std::string output_storage_id = "mcap"; // Output in MCAP format
        
        std::cout << "Input bag format: " << input_storage_id << std::endl;
        std::cout << "Output bag format: " << output_storage_id << std::endl;
        
        // Setup reader
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = input_bag_path_;
        storage_options.storage_id = input_storage_id;
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        reader.open(storage_options, converter_options);

        // Setup writer
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        rosbag2_storage::StorageOptions write_storage_options;
        write_storage_options.uri = output_bag_path_;
        write_storage_options.storage_id = output_storage_id;
        
        writer_->open(write_storage_options, converter_options);

        // Create topic for point cloud
        rosbag2_storage::TopicMetadata cloud_topic;
        cloud_topic.name = "/raw_velodyne_points";
        cloud_topic.type = "sensor_msgs/msg/PointCloud2";
        cloud_topic.serialization_format = "cdr";
        writer_->create_topic(cloud_topic);

        std::cout << "Starting conversion..." << std::endl;
        
        // Process messages
        while (reader.has_next()) {
            auto bag_message = reader.read_next();
            
            if (bag_message->topic_name == lidar_topic_) {
                // Process lidar packets
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                ouster_sensor_msgs::msg::PacketMsg packet_msg;
                
                rclcpp::Serialization<ouster_sensor_msgs::msg::PacketMsg> serialization;
                serialization.deserialize_message(&serialized_msg, &packet_msg);
                
                // Create LidarPacket and process through handler
                ouster::sensor::LidarPacket lidar_packet(packet_msg.buf.size());
                memcpy(lidar_packet.buf.data(), packet_msg.buf.data(), packet_msg.buf.size());
                lidar_packet.host_timestamp = static_cast<uint64_t>(bag_message->recv_timestamp);
                
                current_timestamp_ = bag_message->recv_timestamp;
                
                // Process packet through handler (same as os_cloud_node)
                if (lidar_packet_handler_) {
                    lidar_packet_handler_(lidar_packet);
                }
                
                // Wait for point cloud to be written (backpressure)
                wait_for_processing();
                
            } else {
                // Copy all other messages as-is
                writer_->write(bag_message);
            }
        }
        
        std::cout << "\nConversion complete! Total scans: " << scan_counter_ << std::endl;
    }

private:
    std::string detect_storage_format(const std::string& bag_path) {
        // Check if path points to a .mcap file (C++17 compatible)
        if (bag_path.size() >= 5 && 
            bag_path.substr(bag_path.size() - 5) == ".mcap") {
            return "mcap";
        }
        
        // Check if it's a directory (sqlite3 db3 format)
        if (std::filesystem::is_directory(bag_path)) {
            return "sqlite3";
        }
        
        // Check for specific file in directory
        std::filesystem::path p(bag_path);
        if (std::filesystem::exists(p / "metadata.yaml")) {
            return "sqlite3";
        }
        
        // Default to mcap if file exists
        if (std::filesystem::exists(bag_path)) {
            return "mcap";
        }
        
        throw std::runtime_error("Cannot detect bag format for: " + bag_path);
    }
    
    void setup_processors() {
        int num_returns = ouster_ros::get_n_returns(info_);
        
        std::vector<ouster_ros::LidarScanProcessor> processors;
        
        // Setup point cloud processor (same parameters as os_cloud_node)
        bool organized = true;
        bool destagger = true;
        uint32_t min_range = 0;      // in mm
        uint32_t max_range = 1000000; // in mm (1000m)
        int v_reduction = 1;
        std::string mask_path = "";
        
        processors.push_back(
            ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
                point_type_,
                info_, 
                "velodyne",  // frame_id
                false,       // apply_lidar_to_sensor_transform
                organized, 
                destagger, 
                min_range, 
                max_range, 
                v_reduction, 
                mask_path,
                [this](ouster_ros::PointCloudProcessor_OutputType msgs) {
                    this->handle_point_clouds(msgs);
                }
            )
        );
        
        // Create lidar packet handler with smaller queue (reduce buffering)
        double ptp_utc_tai_offset = -37.0;
        double min_scan_valid_columns_ratio = 0.0;
        
        lidar_packet_handler_ = ouster_ros::LidarPacketHandler::create(
            info_, 
            processors, 
            timestamp_mode_,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            min_scan_valid_columns_ratio
        );
    }
    
    void wait_for_processing() {
        std::unique_lock<std::mutex> lock(processing_mutex_);
        // Give processing thread time to catch up
        processing_cv_.wait_for(lock, std::chrono::milliseconds(1));
    }
    
    void handle_point_clouds(ouster_ros::PointCloudProcessor_OutputType msgs) {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        // Process each return (usually just one for standard processing)
        for (size_t i = 0; i < msgs.size(); ++i) {
            auto& cloud_msg = msgs[i];
            
            // Serialize and write point cloud to bag
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
            auto serialized_cloud = std::make_shared<rclcpp::SerializedMessage>();
            cloud_serialization.serialize_message(cloud_msg.get(), serialized_cloud.get());
            
            auto cloud_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            cloud_bag_msg->topic_name = "/raw_velodyne_points";
            cloud_bag_msg->serialized_data = 
                std::shared_ptr<rcutils_uint8_array_t>(
                    &serialized_cloud->get_rcl_serialized_message(),
                    [serialized_cloud](rcutils_uint8_array_t*) {});
            cloud_bag_msg->recv_timestamp = current_timestamp_;
            
            writer_->write(cloud_bag_msg);
            
            scan_counter_++;
            std::cout << "Converted scan " << scan_counter_ << "\r" << std::flush;
        }
        
        processing_cv_.notify_all();
    }

    std::string input_bag_path_;
    std::string output_bag_path_;
    std::string metadata_file_;
    std::string lidar_topic_;
    std::string point_type_;
    std::string timestamp_mode_;
    
    ouster::sensor::sensor_info info_;
    ouster_ros::LidarPacketHandler::HandlerType lidar_packet_handler_;
    
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    int64_t current_timestamp_;
    int scan_counter_ = 0;
    
    // Synchronization primitives for backpressure
    std::mutex processing_mutex_;
    std::condition_variable processing_cv_;
    bool processing_complete_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <input_bag> <output_bag> <metadata_json> [lidar_topic] [point_type]" 
                  << std::endl;
        std::cerr << "point_type options: original, native, xyz, xyzi, xyzir (default: original)" << std::endl;
        return 1;
    }
    
    std::string input_bag = argv[1];
    std::string output_bag = argv[2];
    std::string metadata_file = argv[3];
    std::string lidar_topic = (argc > 4) ? argv[4] : "/lidar_packets";
    std::string point_type = (argc > 5) ? argv[5] : "original";
    
    try {
        OfflinePacketConverter converter(input_bag, output_bag, metadata_file, 
                                        lidar_topic, point_type);
        converter.convert();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
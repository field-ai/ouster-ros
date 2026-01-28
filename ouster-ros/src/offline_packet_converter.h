#ifndef OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_
#define OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ouster_sensor_msgs/msg/packet_msg.hpp>

// Ouster SDK includes
#include <ouster/types.h>
#include <ouster/lidar_scan.h>

#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "point_cloud_processor_factory.h"

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

class OfflinePacketConverter {
    public:
        OfflinePacketConverter(const std::string& input_bag_dir, 
                            const std::string& ouster_metadata_file,
                            const std::string& robot_name,
                            const std::string& timestamp_mode = "TIME_FROM_PTP_1588");
        
        void convert();

    private:
        std::vector<std::string> getInputBags(const std::string& bag_path);

        bool isMcapBag(const std::string& bag_path);

        void processSingleBag(const std::string& bag_file, 
                            const std::string& storage_id,
                            const rosbag2_cpp::ConverterOptions& converter_options);
        
        void setupProcessors();

        void waitForProcessing();

        void handlePointClouds(ouster_ros::PointCloudProcessor_OutputType msgs);

        std::string getOutputBagDir(const std::string& input_bag_dir);

        std::string getOutputBagFilename(const std::string& input_bag_filename);

        std::string replaceRawWithPointcloud(const std::string& filename);

        ouster::sensor::sensor_info loadOusterMetadata(const std::string& metadata_file);

        std::string input_bag_dir_;
        std::string output_bag_dir_;
        std::string input_lidar_topic_;
        std::string output_lidar_topic_;
        std::string frame_id_;
        std::string robot_name_;
        std::string timestamp_mode_;
        
        std::vector<std::string> input_rosbags_; 
        
        ouster::sensor::sensor_info ouster_metadata_;
        ouster_ros::LidarPacketHandler::HandlerType lidar_packet_handler_;
        
        std::unique_ptr<rosbag2_cpp::Writer> writer_;
        int64_t current_timestamp_;
        int scan_counter_ = 0;
        
        // Synchronization primitives for backpressure
        std::mutex processing_mutex_;
        std::condition_variable processing_cv_;
        bool processing_complete_;

};
#endif  // OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_
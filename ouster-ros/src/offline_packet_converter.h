#ifndef OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_
#define OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "point_cloud_processor.h"
#include <ouster/types.h>

#include <memory>
#include <string>
#include <vector>

constexpr size_t MAX_BAGFILE_SIZE_BYTES = 500ULL * 1024ULL * 1024ULL;  // 500MB
constexpr size_t MAX_CACHE_SIZE_BYTES = 64ULL * 1024ULL * 1024ULL;    // 64MB
constexpr uint64_t NANOSECONDS_PER_SECOND = 1000000000ULL;

class OfflinePacketConverter {
    public:
        OfflinePacketConverter(const std::string& input_bag_dir, 
                            const std::string& ouster_metadata_file,
                            const std::string& robot_name,
                            const std::string& timestamp_mode = "TIME_FROM_PTP_1588");
        ~OfflinePacketConverter();
        
        void convert();

    private:

        bool validateInputs(const std::string& input_bag_dir,
                    const std::string& robot_name,
                    const std::string& timestamp_mode);

        bool validateInputBag(const std::string& bag_path);

        void processSingleBag(const std::string& bag_file, 
                            const std::string& storage_id,
                            const rosbag2_cpp::ConverterOptions& converter_options);        

        std::string getOutputBagDir(const std::string& input_bag_dir);

        std::string replaceRawWithPointcloud(const std::string& filename);

        ouster::sensor::sensor_info loadOusterMetadata(const std::string& metadata_file);

        void writePointClouds(ouster_ros::PointCloudProcessor_OutputType& msgs);

        std::string input_bag_dir_;
        std::string robot_name_;
        std::string timestamp_mode_;
        std::string input_lidar_topic_;
        std::string output_lidar_topic_;
        std::string frame_id_;
        std::string output_bag_dir_;

        ouster::sensor::sensor_info ouster_metadata_;
        
        std::unique_ptr<rosbag2_cpp::Writer> writer_;
        int scan_counter_ = 0;

};
#endif  // OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_
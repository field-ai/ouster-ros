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
                            const std::string& robot_name);
        ~OfflinePacketConverter();
    
        /**
         * @brief Convert the input bag file containing Ouster packets to a new bag file
         *        containing point cloud messages.
         */
        void convert();

    private:

        /**
         * @brief Validate the input parameters.
         * @param input_bag_dir The input bag directory.
         * @param robot_name The robot name.
         * @param timestamp_mode The timestamp mode.
         * @return true if the inputs are valid, false otherwise.
         */
        bool validateInputs(const std::string& input_bag_dir,
                    const std::string& robot_name);
        
        /**
         * @brief Validate the input bag file format.
         * @param bag_dir The input bag directory.
         * @return true if the bag file format is supported, false otherwise.
         */
        bool validateInputBag(const std::string& bag_dir);

        /**
         * @brief Process the input bag.
         * @param bag_dir The input bag directory.
         * @param storage_id The storage ID.
         * @param converter_options The converter options.
         */
        void processBag(const std::string& bag_dir, 
                            const std::string& storage_id,
                            const rosbag2_cpp::ConverterOptions& converter_options);        
        
        /**
         * @brief Get the output bag directory path based on the input bag directory.
         * @param input_bag_dir The input bag directory.
         * @return The output bag directory path.
         */
        std::string getOutputBagDir(const std::string& input_bag_dir);

        /**
         * @brief Replace "raw" with "pointcloud" in the given filename.
         * @param filename The input filename.
         * @return The modified filename with "raw" replaced by "pointcloud".
         */
        std::string replaceRawWithPointcloud(const std::string& filename);

        /**
         * @brief Load Ouster sensor metadata from a metadata file.
         * @param metadata_file The path to the ouster metadata file.
         * @return The loaded Ouster sensor_info.
         */
        ouster::sensor::sensor_info loadOusterMetadata(const std::string& metadata_file);
        
        /**
         * @brief Write point cloud messages to the output bag file.
         * @param msgs The point cloud messages to write.
         */
        void writePointClouds(ouster_ros::PointCloudProcessor_OutputType& msgs);

        /**
         * @brief Input bag directory path.
         */
        std::string input_bag_dir_;

        /**
         * @brief Robot name.
         */
        std::string robot_name_;

        /**
         * @brief Timestamp mode.
         */
        std::string timestamp_mode_;

        /**
         * @brief Input lidar topic name.
         */
        std::string input_lidar_topic_;

        /**
         * @brief Output lidar topic name.
         */
        std::string output_lidar_topic_;

        /**
         * @brief Frame ID for point cloud messages.
         */
        std::string frame_id_;

        /**
         * @brief Output bag directory path.
         */
        std::string output_bag_dir_;

        /**
         * @brief Ouster sensor metadata.
         */
        ouster::sensor::sensor_info ouster_metadata_;
        
        /**
         * @brief Writer for the output bag file.
         */
        std::unique_ptr<rosbag2_cpp::Writer> writer_;

        /**
         * @brief Counter for the number of scans processed.
         */
        int scan_counter_ = 0;
};
#endif  // OUSTER_ROS__OFFLINE_PACKET_CONVERTER_H_
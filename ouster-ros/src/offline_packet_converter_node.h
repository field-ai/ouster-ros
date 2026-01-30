#ifndef OUSTER_ROS__OFFLINE_PACKET_CONVERTER_NODE_H_
#define OUSTER_ROS__OFFLINE_PACKET_CONVERTER_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_transport/record_options.hpp>

#include "point_cloud_processor.h"
#include "point_cloud_processor_factory.h"
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <ouster_sensor_msgs/msg/packet_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <regex>

constexpr size_t MAX_BAGFILE_SIZE_BYTES = 500ULL * 1024ULL * 1024ULL; // 500MB
constexpr size_t MAX_CACHE_SIZE_BYTES = 64ULL * 1024ULL * 1024ULL;    // 64MB
constexpr uint64_t NANOSECONDS_PER_SECOND = 1000000000ULL;
constexpr const char* LIDAR_BAG_PATTERN = "(.*)_lidar_(.*)";
constexpr const char* LIDAR_POINTCLOUD_REPLACE = "$1_lidar_pointcloud_$2";

class OfflinePacketConverterNode : public rclcpp::Node {
public:
  explicit OfflinePacketConverterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OfflinePacketConverterNode();

  /**
   * @brief Convert the input bag file containing Ouster packets to a new bag file
   *        containing point cloud messages.
   */
  void convert();

private:
  /**
   * @brief Initialize the node by setting up paths and validating inputs.
   * @return true if initialization is successful, false otherwise.
   */
  bool init();

  /**
   * @brief Setup the reader and writer for rosbag2.
   * @return true if setup is successful, false otherwise.
   */
  bool setupReaderWriter();

  /**
   * @brief Setup ROS parameters.
   */
  void setupParameters();

  /**
   * @brief Validate the input bag file format.
   * @param bag_dir The input bag directory.
   * @return true if the bag file format is supported, false otherwise.
   */
  bool validateInputBagDir(const std::string& bag_dir);

  /**
   * @brief Process the bag dir.
   */
  bool process();

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
   * @brief Find directories matching a regex pattern within a search directory.
   * @param search_dir The directory to search in.
   * @param pattern The regex pattern to match directory names.
   * @return A map of matched directory indices to their paths.
   */
  std::map<int, std::string> findDirsByRegex(const std::string& search_dir, const std::string& pattern);

  /**
   * @brief Check if the given path is a directory.
   * @param dir_path The path to check.
   * @return true if the path is a directory, false otherwise.
   */
  bool isDirectory(const std::filesystem::path& dir_path);

  /**
   * @brief Check if the given path is a regular file.
   * @param file_path The path to check.
   * @return true if the path is a regular file, false otherwise.
   */
  bool isFile(const std::filesystem::path& file_path);

  /**
   * @brief Deserialize a LidarPacket from a SerializedBagMessage.
   * @param bag_message The serialized bag message.
   * @return The deserialized LidarPacket.
   */
  ouster::sensor::LidarPacket deserializeLidarPacket(const rosbag2_storage::SerializedBagMessage& bag_message);

  /**
   * @brief Extract the scan timestamp from a LidarScan.
   * @param scan The LidarScan object.
   * @param fallback_timestamp The fallback timestamp to use if no valid timestamp is found.
   * @return The extracted scan timestamp.
   */
  uint64_t extractScanTimestamp(const ouster::LidarScan& scan, uint64_t fallback_timestamp);

  /**
   * @brief Input base directory path.
   */
  std::string base_dir_;

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
   * @brief bag reader
   */
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  /**
   * @brief scan counter
   */
  int scan_counter_;

  /**
   * @brief pointcloud procesor params
   */
  std::string point_type_;
  bool organized_;
  bool destagger_;
  double min_range_mm_;
  double max_range_mm_;
  int rows_step_;
  std::string mask_path_;
  std::string ouster_metadata_filepath_;
  bool apply_lidar_to_sensor_transform_;
};
#endif // OUSTER_ROS__OFFLINE_PACKET_CONVERTER_NODE_H_
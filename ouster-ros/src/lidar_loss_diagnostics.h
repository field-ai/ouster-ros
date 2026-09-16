/**
 * Copyright (c) 2026, Field AI, Inc.
 * All rights reserved.
 *
 * @file lidar_loss_diagnostics.h
 * @brief Publishes lidar packet-loss counters on /diagnostics for os_cloud and os_driver.
 */

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidar_packet_handler.h"
#include "udp_socket_stats.h"

namespace ouster_ros {

// Packet loss is invisible in the point cloud (columns of lost packets are not
// reliably zeroed), so report the batcher's column status and the kernel's
// socket drop counter where site logs can see them.
template <typename NodeT>
class LidarLossDiagnostics {
   public:
    explicit LidarLossDiagnostics(NodeT* node) : node_(node) {}

    void start(std::shared_ptr<LidarPacketHandler> handler,
               const ouster::sdk::core::SensorInfo& info) {
        handler_ = std::move(handler);
        if (!handler_) return;
        hardware_id_ = std::to_string(info.sn);
        // SensorInfo uses optional-lite, not std::optional; copy the value across.
        lidar_port_.reset();
        if (info.config.udp_port_lidar) lidar_port_ = *info.config.udp_port_lidar;
        cols_per_scan_ = info.format.columns_per_frame;
        last_stats_ = handler_->stats();
        last_socket_drops_.reset();
        pub_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            *node_, "/diagnostics", rclcpp::QoS(10));
        timer_ = node_->create_wall_timer(std::chrono::seconds(1),
                                          [this]() { publish(); });
    }

    void stop() {
        timer_.reset();
        pub_.reset();
        handler_.reset();
    }

   private:
    void publish() {
        if (!handler_ || !pub_) return;
        const auto now_stats = handler_->stats();
        const uint64_t scans = now_stats.scans_completed - last_stats_.scans_completed;
        const uint64_t incomplete = now_stats.scans_incomplete - last_stats_.scans_incomplete;
        const uint64_t missing = now_stats.columns_missing - last_stats_.columns_missing;
        const uint64_t skipped = now_stats.scans_skipped - last_stats_.scans_skipped;
        const uint64_t ring_drops =
            now_stats.packets_dropped_ring_full - last_stats_.packets_dropped_ring_full;
        last_stats_ = now_stats;

        UdpSocketStats sock;
        if (lidar_port_) sock = read_udp_socket_stats(*lidar_port_);
        uint64_t sock_drops = 0;
        if (sock.found) {
            if (last_socket_drops_ && sock.drops >= *last_socket_drops_)
                sock_drops = sock.drops - *last_socket_drops_;
            last_socket_drops_ = sock.drops;
        }

        const double missing_pct =
            scans && cols_per_scan_
                ? 100.0 * static_cast<double>(missing) /
                      static_cast<double>(scans * cols_per_scan_)
                : 0.0;

        using diagnostic_msgs::msg::DiagnosticStatus;
        DiagnosticStatus status;
        status.name = std::string(node_->get_name()) + ": lidar packet loss";
        status.hardware_id = hardware_id_;
        if (incomplete == 0 && skipped == 0 && sock_drops == 0 && ring_drops == 0) {
            status.level = DiagnosticStatus::OK;
            status.message = "all scans complete";
        } else {
            status.level = (scans && 2 * (incomplete + skipped) > scans)
                               ? DiagnosticStatus::ERROR
                               : DiagnosticStatus::WARN;
            std::ostringstream msg;
            msg << incomplete << " incomplete, " << skipped << " skipped of " << scans
                << " scans; " << missing << " columns missing; " << sock_drops
                << " socket drops; " << ring_drops << " ring-buffer drops";
            status.message = msg.str();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "lidar packet loss: %s", status.message.c_str());
        }
        auto add = [&status](const char* key, const std::string& value) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        };
        add("scans", std::to_string(scans));
        add("incomplete_scans", std::to_string(incomplete));
        add("skipped_scans", std::to_string(skipped));
        add("missing_columns", std::to_string(missing));
        add("missing_columns_pct", std::to_string(missing_pct));
        add("ring_buffer_drops", std::to_string(ring_drops));
        add("socket_rx_drops", std::to_string(sock_drops));
        add("socket_rx_drops_total", std::to_string(sock.drops));
        add("socket_rx_queue_bytes", std::to_string(sock.rx_queue_bytes));
        add("incomplete_scans_total", std::to_string(now_stats.scans_incomplete));
        add("missing_columns_total", std::to_string(now_stats.columns_missing));

        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = node_->now();
        array.status.push_back(status);
        pub_->publish(array);
    }

    NodeT* node_;
    std::shared_ptr<LidarPacketHandler> handler_;
    std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LidarPacketHandler::Stats last_stats_{};
    std::optional<uint64_t> last_socket_drops_;
    std::optional<uint16_t> lidar_port_;
    size_t cols_per_scan_ = 0;
    std::string hardware_id_;
};

}  // namespace ouster_ros

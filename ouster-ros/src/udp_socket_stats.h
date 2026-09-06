/**
 * Copyright (c) 2026, Field AI, Inc.
 * All rights reserved.
 *
 * @file udp_socket_stats.h
 * @brief Kernel-side receive statistics for the sensor's UDP socket.
 */

#pragma once

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace ouster_ros {

struct UdpSocketStats {
    bool found = false;
    uint64_t rx_queue_bytes = 0;  // bytes waiting in the socket receive buffer
    uint64_t drops = 0;           // datagrams the kernel dropped for lack of buffer
};

// The SDK opens dual-stack sockets, so the lidar socket normally shows up in
// /proc/net/udp6 even though the sensor talks IPv4; check both tables.
inline UdpSocketStats read_udp_socket_stats(uint16_t local_port) {
    UdpSocketStats stats;
    std::ostringstream port_hex;
    port_hex << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << local_port;
    const std::string suffix = ":" + port_hex.str();
    for (const char* table : {"/proc/net/udp6", "/proc/net/udp"}) {
        std::ifstream in(table);
        std::string line;
        std::getline(in, line);  // header
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string sl, local, remote, st, queues;
            fields >> sl >> local >> remote >> st >> queues;
            if (local.size() < suffix.size() ||
                local.compare(local.size() - suffix.size(), suffix.size(), suffix) != 0) {
                continue;
            }
            // Remaining columns: tr tm->when retrnsmt uid timeout inode ref pointer drops
            std::string tok, last;
            while (fields >> tok) last = tok;
            const auto colon = queues.find(':');
            if (colon == std::string::npos) continue;
            stats.found = true;
            stats.rx_queue_bytes = std::stoull(queues.substr(colon + 1), nullptr, 16);
            stats.drops = last.empty() ? 0 : std::stoull(last);
            return stats;
        }
    }
    return stats;
}

}  // namespace ouster_ros

#pragma once

#include <string>

namespace openarm_wifi_teleop {
namespace utils {

// Result of resolving which local interface routes to a peer and what kind of
// link it is. Read from sysfs; no packets are sent.
struct LinkInfo {
    bool resolved = false;      // route + interface lookup succeeded
    std::string ifname;         // e.g. "eth0"
    std::string local_ip;       // source address the kernel picked for the peer
    bool is_loopback = false;   // peer is reached over "lo"
    bool is_wireless = false;   // /sys/class/net/<if>/wireless exists (or SIOCGIWNAME)
    bool has_device = false;    // /sys/class/net/<if>/device exists (physical NIC, not virtual)
    bool carrier = false;       // /sys/class/net/<if>/carrier == 1
    int speed_mbps = -1;        // /sys/class/net/<if>/speed, -1 if unknown
    std::string error;          // why resolution failed

    std::string describe() const;
};

// Resolve the interface the kernel would use to reach peer_ip (via a connected
// UDP socket + getsockname, so it follows the routing table) and read its
// sysfs attributes.
LinkInfo query_link_to(const std::string& peer_ip);

} // namespace utils
} // namespace openarm_wifi_teleop

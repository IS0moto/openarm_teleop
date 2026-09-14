#include "openarm_wifi_teleop/utils/link_check.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>

namespace openarm_wifi_teleop {
namespace utils {

namespace {

bool path_exists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

bool read_sysfs_int(const std::string& path, long& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    if (!std::getline(f, line)) return false;
    try {
        out = std::stol(line);
    } catch (...) {
        return false;
    }
    return true;
}

// SIOCGIWNAME is the wireless-extensions "is this a wireless device" probe.
// Not all drivers implement it (nl80211-only drivers), so sysfs is primary.
bool ioctl_is_wireless(const std::string& ifname) {
    constexpr unsigned long SIOCGIWNAME_ = 0x8B01;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    char req[64];
    std::memset(req, 0, sizeof(req));
    std::strncpy(req, ifname.c_str(), IFNAMSIZ - 1);
    bool wireless = ioctl(fd, SIOCGIWNAME_, req) == 0;
    close(fd);
    return wireless;
}

} // namespace

std::string LinkInfo::describe() const {
    std::ostringstream os;
    if (!resolved) {
        os << "unresolved (" << error << ")";
        return os.str();
    }
    os << ifname << " local_ip=" << local_ip
       << (is_loopback ? " loopback" : "")
       << (is_wireless ? " wireless" : " wired")
       << (has_device ? "" : " virtual")
       << " carrier=" << (carrier ? "1" : "0")
       << " speed=" << speed_mbps << "Mbps";
    return os.str();
}

LinkInfo query_link_to(const std::string& peer_ip) {
    LinkInfo info;

    struct sockaddr_in peer;
    std::memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons(9);  // discard; connect() on UDP sends nothing
    if (inet_pton(AF_INET, peer_ip.c_str(), &peer.sin_addr) <= 0) {
        info.error = "invalid peer ip: " + peer_ip;
        return info;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        info.error = std::string("socket: ") + strerror(errno);
        return info;
    }
    if (connect(fd, reinterpret_cast<struct sockaddr*>(&peer), sizeof(peer)) < 0) {
        info.error = std::string("no route to ") + peer_ip + ": " + strerror(errno);
        close(fd);
        return info;
    }
    struct sockaddr_in local;
    socklen_t len = sizeof(local);
    if (getsockname(fd, reinterpret_cast<struct sockaddr*>(&local), &len) < 0) {
        info.error = std::string("getsockname: ") + strerror(errno);
        close(fd);
        return info;
    }
    close(fd);

    char buf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &local.sin_addr, buf, sizeof(buf));
    info.local_ip = buf;

    struct ifaddrs* ifs = nullptr;
    if (getifaddrs(&ifs) != 0) {
        info.error = std::string("getifaddrs: ") + strerror(errno);
        return info;
    }
    for (struct ifaddrs* it = ifs; it != nullptr; it = it->ifa_next) {
        if (!it->ifa_addr || it->ifa_addr->sa_family != AF_INET) continue;
        auto* a = reinterpret_cast<struct sockaddr_in*>(it->ifa_addr);
        if (a->sin_addr.s_addr == local.sin_addr.s_addr) {
            info.ifname = it->ifa_name;
            info.is_loopback = (it->ifa_flags & IFF_LOOPBACK) != 0;
            break;
        }
    }
    freeifaddrs(ifs);

    if (info.ifname.empty()) {
        info.error = "no interface owns " + info.local_ip;
        return info;
    }

    const std::string sys = "/sys/class/net/" + info.ifname;
    info.is_wireless = path_exists(sys + "/wireless") || path_exists(sys + "/phy80211") ||
                       ioctl_is_wireless(info.ifname);
    info.has_device = path_exists(sys + "/device");
    long v = 0;
    info.carrier = read_sysfs_int(sys + "/carrier", v) && v == 1;
    if (info.is_loopback) info.carrier = true;
    info.speed_mbps = read_sysfs_int(sys + "/speed", v) ? static_cast<int>(v) : -1;

    info.resolved = true;
    return info;
}

} // namespace utils
} // namespace openarm_wifi_teleop

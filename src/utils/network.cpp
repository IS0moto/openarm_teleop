#include "openarm_wifi_teleop/utils/network.hpp"
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

namespace openarm_wifi_teleop {
namespace utils {

std::string get_interface_ip(const std::string& interface_name) {
    struct ifaddrs *ifaddr, *ifa;
    std::string ip = "";

    if (getifaddrs(&ifaddr) == -1) {
        return "";
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;

        if (ifa->ifa_addr->sa_family == AF_INET) { // IPv4
            if (interface_name == ifa->ifa_name) {
                char addr[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr, addr, INET_ADDRSTRLEN);
                ip = addr;
                break;
            }
        }
    }

    freeifaddrs(ifaddr);
    return ip;
}

}
}

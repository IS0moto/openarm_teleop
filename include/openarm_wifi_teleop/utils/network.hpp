#pragma once
#include <string>

namespace openarm_wifi_teleop {
namespace utils {

/**
 * @brief Get the IP address of a network interface
 * @param interface_name Name of the interface (e.g., "wlp46s0")
 * @return IP address as string, or empty string if not found
 */
std::string get_interface_ip(const std::string& interface_name);

}
}

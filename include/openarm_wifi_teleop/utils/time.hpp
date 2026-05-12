#pragma once

#include <chrono>
#include <cstdint>

namespace openarm_wifi_teleop {
namespace utils {

inline uint64_t now_ns() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

inline uint64_t system_now_ns() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace utils
} // namespace openarm_wifi_teleop

#pragma once

#include <iostream>
#include <string>
#include <sstream>

namespace openarm_wifi_teleop {
namespace utils {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

void log(LogLevel level, const std::string& msg);

#define LOG_DEBUG(msg) do { std::stringstream ss; ss << msg; openarm_wifi_teleop::utils::log(openarm_wifi_teleop::utils::LogLevel::DEBUG, ss.str()); } while(0)
#define LOG_INFO(msg)  do { std::stringstream ss; ss << msg; openarm_wifi_teleop::utils::log(openarm_wifi_teleop::utils::LogLevel::INFO,  ss.str()); } while(0)
#define LOG_WARN(msg)  do { std::stringstream ss; ss << msg; openarm_wifi_teleop::utils::log(openarm_wifi_teleop::utils::LogLevel::WARN,  ss.str()); } while(0)
#define LOG_ERROR(msg) do { std::stringstream ss; ss << msg; openarm_wifi_teleop::utils::log(openarm_wifi_teleop::utils::LogLevel::ERROR, ss.str()); } while(0)
#define LOG_FATAL(msg) do { std::stringstream ss; ss << msg; openarm_wifi_teleop::utils::log(openarm_wifi_teleop::utils::LogLevel::FATAL, ss.str()); } while(0)

} // namespace utils
} // namespace openarm_wifi_teleop

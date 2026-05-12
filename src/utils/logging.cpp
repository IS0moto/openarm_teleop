#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include <mutex>
#include <iomanip>

namespace openarm_wifi_teleop {
namespace utils {

static std::mutex log_mutex;

void log(LogLevel level, const std::string& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    std::string level_str;
    switch (level) {
        case LogLevel::DEBUG: level_str = "DEBUG"; break;
        case LogLevel::INFO:  level_str = "INFO "; break;
        case LogLevel::WARN:  level_str = "WARN "; break;
        case LogLevel::ERROR: level_str = "ERROR"; break;
        case LogLevel::FATAL: level_str = "FATAL"; break;
    }
    
    if (level == LogLevel::ERROR || level == LogLevel::FATAL) {
        std::cerr << "[" << now_ms << "] [" << level_str << "] " << msg << std::endl;
    } else {
        std::cout << "[" << now_ms << "] [" << level_str << "] " << msg << std::endl;
    }
}

} // namespace utils
} // namespace openarm_wifi_teleop

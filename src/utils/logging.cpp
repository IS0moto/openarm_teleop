#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <string>

namespace openarm_wifi_teleop {
namespace utils {

static std::mutex log_mutex;

// Minimum level to emit. Default INFO; override with env OPENARM_LOG_LEVEL
// (DEBUG|INFO|WARN|ERROR|FATAL). Messages below this are dropped.
static LogLevel parse_log_level(const char* s) {
    std::string v = s ? s : "";
    for (auto& c : v) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (v == "DEBUG") return LogLevel::DEBUG;
    if (v == "WARN" || v == "WARNING") return LogLevel::WARN;
    if (v == "ERROR") return LogLevel::ERROR;
    if (v == "FATAL") return LogLevel::FATAL;
    return LogLevel::INFO;
}

static LogLevel min_log_level() {
    static const LogLevel level = parse_log_level(std::getenv("OPENARM_LOG_LEVEL"));
    return level;
}

void log(LogLevel level, const std::string& msg) {
    if (level < min_log_level()) return;
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

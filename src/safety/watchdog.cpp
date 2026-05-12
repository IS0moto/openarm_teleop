#include "openarm_wifi_teleop/safety/watchdog.hpp"

namespace openarm_wifi_teleop {
namespace safety {

Watchdog::Watchdog(double warning_ms, double hold_ms, double disable_ms)
    : warning_ms_(warning_ms), hold_ms_(hold_ms), disable_ms_(disable_ms) {}

Watchdog::Status Watchdog::check(double time_since_last_packet_ms) {
    if (time_since_last_packet_ms >= disable_ms_) {
        return Status::DISABLE;
    } else if (time_since_last_packet_ms >= hold_ms_) {
        return Status::HOLD;
    } else if (time_since_last_packet_ms >= warning_ms_) {
        return Status::WARNING;
    }
    return Status::OK;
}

}
}

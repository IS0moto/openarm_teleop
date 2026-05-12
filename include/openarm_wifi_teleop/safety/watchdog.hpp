#pragma once

namespace openarm_wifi_teleop {
namespace safety {

class Watchdog {
public:
    enum class Status {
        OK,
        WARNING,
        HOLD,
        DISABLE
    };

    Watchdog(double warning_ms, double hold_ms, double disable_ms);
    
    Status check(double time_since_last_packet_ms);

private:
    double warning_ms_;
    double hold_ms_;
    double disable_ms_;
};

}
}

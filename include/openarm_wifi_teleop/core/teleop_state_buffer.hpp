#pragma once
#include "openarm_wifi_teleop/net/teleop_packet.hpp"
#include <mutex>

namespace openarm_wifi_teleop {
namespace core {

class TeleopStateBuffer {
public:
    TeleopStateBuffer();

    void update(const net::TeleopPacket& packet);
    bool get_latest(net::TeleopPacket& out_packet);

    uint64_t get_last_receive_time_ns() const;
    uint32_t get_last_seq() const;
    uint32_t get_lost_packet_count() const;
    double get_average_receive_period_ms() const;
    double get_max_receive_gap_ms() const;

private:
    mutable std::mutex mutex_;
    net::TeleopPacket latest_packet_;
    bool has_data_;

    uint64_t last_receive_time_ns_;
    uint32_t last_seq_;
    uint32_t lost_packet_count_;

    double average_receive_period_ms_;
    double max_receive_gap_ms_;
    
    static constexpr double alpha_ = 0.1; // For EMA of period
};

}
}

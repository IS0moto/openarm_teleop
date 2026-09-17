#pragma once
#include "openarm_wifi_teleop/net/feedback_packet.hpp"
#include <cstdint>
#include <deque>
#include <mutex>

namespace openarm_wifi_teleop {
namespace core {

// Latest follower feedback for one arm plus the link statistics the bilateral
// gate needs (rate, loss, round trip) over a sliding window.
class FeedbackStateBuffer {
public:
    struct Stats {
        bool has_data = false;
        double age_ms = -1.0;          // time since the newest packet arrived
        double rate_hz = 0.0;          // packets received in the window / window
        double loss_percent = 0.0;     // seq gaps in the window
        double rtt_p50_ms = -1.0;      // command send -> feedback receive, leader clock
        double rtt_p95_ms = -1.0;
        double rtt_max_ms = -1.0;
        double cmd_hold_p95_ms = -1.0; // follower-side command age when feedback was sent
        uint32_t received_total = 0;
        uint32_t lost_total = 0;
    };

    explicit FeedbackStateBuffer(double window_s = 1.0);

    // Called from the receive thread. rtt_ns is measured by the caller
    // (system clock now - echo_cmd_send_time_ns); pass 0 if not measurable.
    void update(const net::FeedbackPacket& packet, uint64_t rtt_ns);

    bool get_latest(net::FeedbackPacket& out) const;
    Stats snapshot() const;
    void reset();

private:
    struct Sample {
        uint64_t recv_time_ns;
        uint32_t lost_before;
        uint64_t rtt_ns;
        uint64_t cmd_hold_ns;
    };

    void trim_locked(uint64_t now_ns);

    mutable std::mutex mutex_;
    uint64_t window_ns_;
    net::FeedbackPacket latest_{};
    bool has_data_ = false;
    uint64_t last_receive_time_ns_ = 0;
    uint32_t last_seq_ = 0;
    uint32_t received_total_ = 0;
    uint32_t lost_total_ = 0;
    std::deque<Sample> samples_;
};

}
}

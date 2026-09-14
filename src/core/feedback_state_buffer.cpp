#include "openarm_wifi_teleop/core/feedback_state_buffer.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"

#include <algorithm>
#include <vector>

namespace openarm_wifi_teleop {
namespace core {

FeedbackStateBuffer::FeedbackStateBuffer(double window_s)
    : window_ns_(static_cast<uint64_t>(window_s * 1e9)) {}

void FeedbackStateBuffer::update(const net::FeedbackPacket& packet, uint64_t rtt_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t now_ns = utils::now_ns();

    uint32_t lost_before = 0;
    if (has_data_) {
        uint32_t diff = packet.seq - last_seq_;
        if (diff > 0x7FFFFFFF) {
            // Sequence went backwards (sender restarted). Start over.
            LOG_WARN("Feedback sequence jump (" << last_seq_ << " -> " << packet.seq << "). Resetting.");
            samples_.clear();
        } else if (diff == 0) {
            return;  // duplicate
        } else if (diff > 1) {
            lost_before = diff - 1;
            lost_total_ += lost_before;
        }
    }

    latest_ = packet;
    last_seq_ = packet.seq;
    last_receive_time_ns_ = now_ns;
    has_data_ = true;
    received_total_++;
    samples_.push_back({now_ns, lost_before, rtt_ns, packet.cmd_hold_ns});
    trim_locked(now_ns);
}

void FeedbackStateBuffer::trim_locked(uint64_t now_ns) {
    while (!samples_.empty() && now_ns - samples_.front().recv_time_ns > window_ns_) {
        samples_.pop_front();
    }
}

bool FeedbackStateBuffer::get_latest(net::FeedbackPacket& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) return false;
    out = latest_;
    return true;
}

FeedbackStateBuffer::Stats FeedbackStateBuffer::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats s;
    s.has_data = has_data_;
    s.received_total = received_total_;
    s.lost_total = lost_total_;
    if (!has_data_) return s;

    const uint64_t now_ns = utils::now_ns();
    s.age_ms = (now_ns - last_receive_time_ns_) / 1e6;

    // Count only samples inside the window (trim is done on update; a stalled
    // link would otherwise keep stale samples alive here).
    std::vector<uint64_t> rtts, holds;
    uint32_t received = 0, lost = 0;
    for (const auto& smp : samples_) {
        if (now_ns - smp.recv_time_ns > window_ns_) continue;
        received++;
        lost += smp.lost_before;
        if (smp.rtt_ns > 0) rtts.push_back(smp.rtt_ns);
        holds.push_back(smp.cmd_hold_ns);
    }
    s.rate_hz = received / (window_ns_ / 1e9);
    s.loss_percent = (received + lost) > 0 ? 100.0 * lost / (received + lost) : 0.0;

    auto percentile = [](std::vector<uint64_t>& v, double p) {
        std::sort(v.begin(), v.end());
        size_t idx = static_cast<size_t>(p * (v.size() - 1) + 0.5);
        return v[std::min(idx, v.size() - 1)] / 1e6;
    };
    if (!rtts.empty()) {
        s.rtt_p50_ms = percentile(rtts, 0.50);
        s.rtt_p95_ms = percentile(rtts, 0.95);
        s.rtt_max_ms = rtts.back() / 1e6;
    }
    if (!holds.empty()) {
        s.cmd_hold_p95_ms = percentile(holds, 0.95);
    }
    return s;
}

void FeedbackStateBuffer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    has_data_ = false;
    samples_.clear();
}

}
}

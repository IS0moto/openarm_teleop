#include "openarm_wifi_teleop/core/teleop_state_buffer.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"

namespace openarm_wifi_teleop {
namespace core {

TeleopStateBuffer::TeleopStateBuffer()
    : has_data_(false), last_receive_time_ns_(0), last_seq_(0),
      lost_packet_count_(0), average_receive_period_ms_(0.0), max_receive_gap_ms_(0.0) {
}

void TeleopStateBuffer::update(const net::TeleopPacket& packet) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    uint64_t now_ns = utils::now_ns();
    
    if (has_data_) {
        // Out of order or duplicate packet rejection
        // Sequence number wraps around at uint32_t, we need to handle it.
        uint32_t diff = packet.seq - last_seq_;
        if (diff > 0x7FFFFFFF) {
            // Sequence jumped backwards significantly (e.g. restart)
            LOG_WARN("Sequence jump detected (" << last_seq_ << " -> " << packet.seq << "). Resetting buffer.");
            has_data_ = false;
            average_receive_period_ms_ = 0.0;
            max_receive_gap_ms_ = 0.0;
            // fall through to initialization logic below
        } else {
            // Detect loss
            if (diff > 1) {
                lost_packet_count_ += (diff - 1);
            }

            // Calculate timing stats
            double gap_ms = (now_ns - last_receive_time_ns_) / 1e6;
            if (gap_ms > max_receive_gap_ms_) {
                max_receive_gap_ms_ = gap_ms;
            }
            
            if (average_receive_period_ms_ == 0.0) {
                average_receive_period_ms_ = gap_ms;
            } else {
                average_receive_period_ms_ = alpha_ * gap_ms + (1.0 - alpha_) * average_receive_period_ms_;
            }
        }
    }

    // Update packet data (both initial and subsequent packets)
    latest_packet_ = packet;
    last_seq_ = packet.seq;
    last_receive_time_ns_ = now_ns;
    has_data_ = true;
}

bool TeleopStateBuffer::get_latest(net::TeleopPacket& out_packet) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) return false;
    out_packet = latest_packet_;
    return true;
}

uint64_t TeleopStateBuffer::get_last_receive_time_ns() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_receive_time_ns_;
}

uint32_t TeleopStateBuffer::get_last_seq() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_seq_;
}

uint32_t TeleopStateBuffer::get_lost_packet_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lost_packet_count_;
}

double TeleopStateBuffer::get_average_receive_period_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return average_receive_period_ms_;
}

double TeleopStateBuffer::get_max_receive_gap_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_receive_gap_ms_;
}

}
}

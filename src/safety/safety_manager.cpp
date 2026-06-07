#include "openarm_wifi_teleop/safety/safety_manager.hpp"
#include "openarm_wifi_teleop/safety/watchdog.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"

namespace openarm_wifi_teleop {
namespace safety {

SafetyManager::SafetyManager(const SafetyConfig& config, const std::string& arm_name)
    : config_(config), state_(SafetyState::DISABLED), arm_name_(arm_name), estop_triggered_(false) {
    if (config_.require_enable) {
        state_ = SafetyState::WAITING_FOR_ENABLE;
    } else {
        state_ = SafetyState::READY;
    }
}

void SafetyManager::update(const net::TeleopPacket& latest_packet, double time_since_last_packet_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (estop_triggered_ || latest_packet.estop == 1) {
        if (state_ != SafetyState::ESTOP) {
            LOG_ERROR("[" << arm_name_ << "] ESTOP triggered!");
        }
        state_ = SafetyState::ESTOP;
        return;
    }

    if (state_ == SafetyState::DISABLED) {
        return;
    }

    if (config_.require_enable && latest_packet.enable == 0) {
        if (state_ != SafetyState::WAITING_FOR_ENABLE) {
            LOG_INFO("[" << arm_name_ << "] Enable signal cleared. Waiting for enable.");
        }
        state_ = SafetyState::WAITING_FOR_ENABLE;
        return;
    }

    Watchdog wd(config_.watchdog_warning_ms, config_.watchdog_hold_ms, config_.watchdog_disable_ms);
    Watchdog::Status wd_status = wd.check(time_since_last_packet_ms);

    if (wd_status == Watchdog::Status::DISABLE) {
        if (state_ != SafetyState::FAULT && state_ != SafetyState::DISABLED) {
            LOG_ERROR("[" << arm_name_ << "] Watchdog timeout! Disabling. Gap: " << time_since_last_packet_ms << "ms");
        }
        state_ = SafetyState::FAULT; // Or DISABLED
        return;
    }

    if (state_ == SafetyState::WAITING_FOR_ENABLE) {
        if (latest_packet.enable == 1) {
            LOG_INFO("[" << arm_name_ << "] Enable signal received. Transitioning to READY.");
            state_ = SafetyState::READY;
        } else {
            return; // Still waiting
        }
    }

    if (state_ == SafetyState::READY || state_ == SafetyState::ACTIVE || state_ == SafetyState::WARNING_TIMEOUT || state_ == SafetyState::HOLD) {
        if (wd_status == Watchdog::Status::HOLD) {
            if (state_ != SafetyState::HOLD) {
                LOG_WARN("[" << arm_name_ << "] Watchdog hold triggered.");
            }
            state_ = SafetyState::HOLD;
        } else if (wd_status == Watchdog::Status::WARNING) {
            state_ = SafetyState::WARNING_TIMEOUT;
        } else {
            state_ = SafetyState::ACTIVE;
        }
    }
}

SafetyState SafetyManager::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void SafetyManager::trigger_estop() {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_triggered_ = true;
    state_ = SafetyState::ESTOP;
    LOG_FATAL("[" << arm_name_ << "] Software ESTOP triggered!");
}

void SafetyManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_triggered_ = false;
    if (config_.require_enable) {
        state_ = SafetyState::WAITING_FOR_ENABLE;
    } else {
        state_ = SafetyState::READY;
    }
    LOG_INFO("SafetyManager reset.");
}

}
}

#pragma once

#include <algorithm>

namespace openarm_wifi_teleop {
namespace control {

// Engage/release state machine for leader-side force rendering on one arm.
//
//   OFF --want--> WAIT_ALIGN --aligned--> RAMP_UP --scale=1--> ENGAGED
//   any state --!want--> RAMP_DOWN --scale=0--> OFF
//
// `want` is "bilateral requested and fresh, usable feedback"; `aligned` is
// "leader and follower poses are within tolerance". Engaging only when aligned
// and ramping the gain avoids a jerk when the two arms start apart (after AI
// mode, for instance). Release ramps down quickly but never steps.
class BilateralEngager {
public:
    struct Config {
        double ramp_up_s = 0.5;
        double ramp_down_s = 0.1;
    };

    enum class State { OFF, WAIT_ALIGN, RAMP_UP, ENGAGED, RAMP_DOWN };

    explicit BilateralEngager(const Config& config) : config_(config) {}

    void update(bool want, bool aligned, double dt_s) {
        switch (state_) {
            case State::OFF:
                scale_ = 0.0;
                if (want) state_ = State::WAIT_ALIGN;
                break;
            case State::WAIT_ALIGN:
                scale_ = 0.0;
                if (!want) state_ = State::OFF;
                else if (aligned) state_ = State::RAMP_UP;
                break;
            case State::RAMP_UP:
                if (!want) { state_ = State::RAMP_DOWN; break; }
                scale_ = std::min(1.0, scale_ + dt_s / std::max(config_.ramp_up_s, 1e-3));
                if (scale_ >= 1.0) state_ = State::ENGAGED;
                break;
            case State::ENGAGED:
                scale_ = 1.0;
                if (!want) state_ = State::RAMP_DOWN;
                break;
            case State::RAMP_DOWN:
                scale_ = std::max(0.0, scale_ - dt_s / std::max(config_.ramp_down_s, 1e-3));
                if (scale_ <= 0.0) state_ = want ? State::WAIT_ALIGN : State::OFF;
                break;
        }
    }

    // Immediate stop (estop / shutdown): no ramp.
    void reset() { state_ = State::OFF; scale_ = 0.0; }

    double scale() const { return scale_; }
    State state() const { return state_; }
    bool active() const { return scale_ > 0.0; }
    bool engaged() const { return state_ == State::ENGAGED; }

    const char* state_name() const {
        switch (state_) {
            case State::OFF: return "off";
            case State::WAIT_ALIGN: return "wait_align";
            case State::RAMP_UP: return "ramp_up";
            case State::ENGAGED: return "engaged";
            case State::RAMP_DOWN: return "ramp_down";
        }
        return "unknown";
    }

private:
    Config config_;
    State state_ = State::OFF;
    double scale_ = 0.0;
};

}
}

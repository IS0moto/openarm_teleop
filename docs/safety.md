# Safety

- startup sequence: Initialize CAN, set up dynamics, adjust to initial position.
- enable sequence: Starts disabled. If `--enable` is set on Leader, it sends `enable=1`. Follower transitions from `WAITING_FOR_ENABLE` to `READY`/`ACTIVE`.
- watchdog:
  - 20ms: WARNING state.
  - 50ms: HOLD state (holds current target).
  - 100ms: DISABLE/FAULT state.
- estop: Ctrl+C or software estop disables all motors immediately.
- rate limit: Target position delta is clamped to `max_velocity * cycle_time`.
- shutdown sequence: Disable all motors (`disable_all()`) safely.

# Safety Model

Safety behavior is implemented mainly in:

```text
src/safety/watchdog.cpp
src/safety/safety_manager.cpp
src/safety/rate_limiter.cpp
```

## Follower Safety States

| State | Meaning |
| --- | --- |
| `DISABLED` | Not controlling motors. |
| `WAITING_FOR_ENABLE` | Valid packets may arrive, but `enable=1` has not been received. |
| `READY` | Enable received; next valid command can enter active control. |
| `ACTIVE` | Normal control. |
| `WARNING_TIMEOUT` | Packet age exceeded warning threshold. |
| `HOLD` | Packet age exceeded hold threshold; hold last reference. |
| `FAULT` | Packet age exceeded disable threshold or safety fault. |
| `ESTOP` | Estop requested. |

## Default Watchdog Timing

From `SafetyConfig`:

```text
warning: 20 ms
hold:    50 ms
disable: 100 ms
```

The bimanual follower exposes:

```bash
--watchdog-hold-ms <ms>
--watchdog-disable-ms <ms>
```

For first real-hardware tests, it is common to use a larger disable threshold:

```bash
--watchdog-disable-ms 1000
```

Then reduce it after network behavior is stable.

## Enable Behavior

Leaders start disabled unless `--enable` is passed.

Follower transition:

```text
WAITING_FOR_ENABLE -> READY -> ACTIVE
```

If command packets stop arriving, the follower transitions toward warning, hold, and fault based on packet age.

## Rate Limiting

The follower limits per-cycle target changes with:

```text
max_target_delta_rad_per_cycle = 0.02
max_joint_velocity_rad_s      = 1.5
```

The effective limit is the stricter of the per-cycle delta and velocity times control period.

VR leader also has its own rate limit in `config/vr_teleop_config.yaml`:

```yaml
rate_limit:
  enabled: true
  max_step_rad:
    right_arm: [...]
    left_arm: [...]
```

## VR-Specific Safety

`wifi_vr_bimanual_leader` only enables outgoing commands when:

- the relevant controller Grip is held
- follower telemetry is fresh
- IK succeeds
- no estop/stale condition is active
- `--dry-run` is not set

Use `--dry-run` for mapping and IK tests.

## Shutdown

Use `Ctrl+C`. Real-hardware programs call `disable_all()` during shutdown paths. If a process is stuck:

```bash
./script/kill_teleop.sh
```

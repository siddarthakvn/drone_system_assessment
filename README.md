# Drone Follower Assessment

ROS 2 system where a PX4 SITL drone in Gazebo Classic takes off to 20 m and follows a moving car. One launch command starts the full stack.

## Requirements Covered

- PX4 SITL drone in Gazebo Classic
- Car drives a repeating circular path and publishes `/car/position`
- Follower node publishes offset waypoints to `/drone/waypoint`
- `px4_manager` arms, takes off, and tracks in offboard mode
- Failure thresholds live in `src/drone_system/config/params.yaml`
- Structured logging via `src/drone_system/drone_system/log_utils.py`
- Telemetry artifacts, plots, log summary, Docker CI workflow

## Prerequisites

Tested on Ubuntu 22.04 with:

- ROS 2 Humble
- Gazebo Classic 11 (`gazebo`, `ros-humble-gazebo-ros-pkgs`)
- `git`, `cmake`, `build-essential`, `python3-venv`, `python3-pip`
- Internet access for first-time PX4 bootstrap

## First-Time Setup

From the repository root:

```bash
cd drone_system_assessment
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
source /opt/ros/humble/setup.bash
bash scripts/bootstrap_px4.sh
python3 -m colcon build --packages-select drone_system
source install/setup.bash
```

`scripts/bootstrap_px4.sh` clones PX4 into `external/PX4-Autopilot`, applies the small Gazebo build patch in `patches/`, and builds SITL once. This step takes several minutes on first run.

## Run The System

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_system full_stack.launch.py
```

Expected behavior:

1. Gazebo opens with the follow world and PX4 iris model
2. Car spawns and drives a circular path
3. Drone arms, climbs to 20 m, then follows at the configured offset
4. Telemetry is written to `artifacts/run_*/telemetry.csv`
5. Structured events are written to `artifacts/run_*/events.log`

### Headless / CI Mode

```bash
ros2 launch drone_system full_stack.launch.py headless:=true
```

## Failure Handling

All thresholds are in `src/drone_system/config/params.yaml`:

| Failure | Parameter | Behavior |
| --- | --- | --- |
| Car position gap > 200 ms | `follower_node.car_position_timeout_s` | Hover at last valid waypoint, log ERROR |
| PX4 arm failure | `px4_manager.arm_retry_limit` | Retry, then shut down cleanly |
| Position jump > 5 m | `follower_node.max_position_jump_m` | Discard update, hold last valid target |
| RTF < 0.8 | `health_monitor.rtf_warning_threshold` | Log WARNING every 5 s until recovery |

## Post-Run Analysis

Find the latest run:

```bash
LATEST_RUN=$(ls -td artifacts/run_* | head -n 1)
echo "$LATEST_RUN"
```

Generate plots:

```bash
source .venv/bin/activate
python3 tools/plot_run.py "$LATEST_RUN/telemetry.csv" "$LATEST_RUN/plots"
```

Summarize structured logs:

```bash
python3 tools/log_summary.py "$LATEST_RUN/events.log"
```

## Integration Test

Local Docker integration test (60 s, headless):

```bash
docker build -t drone-system-integration .
docker run --rm drone-system-integration
```

Or without Docker:

```bash
bash scripts/run_integration_test.sh
```

The test checks that, in the final 30 s of a run:

- drone altitude stayed above 1 m
- no `ERROR` events were logged

## Troubleshooting

If Gazebo/PX4 fail to start because old processes are still running:

```bash
pkill -x px4 || true
pkill -x mavsdk_server || true
pkill -x gzserver || true
pkill -x gzclient || true
```

If plotting fails with `ModuleNotFoundError: matplotlib`, activate the project venv first:

```bash
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

If the drone takes off but does not follow, confirm `px4_manager` logged `takeoff_complete` and inspect the latest `artifacts/run_*/plots/xy_paths.png`.

## Repository Layout

```text
src/drone_system/          ROS 2 package
scripts/bootstrap_px4.sh   first-time PX4 setup
scripts/run_integration_test.sh
scripts/check_integration_run.py
tools/plot_run.py
tools/log_summary.py
.github/workflows/integration_test.yml
ANALYSIS.md
SUBMISSION.md
```

## Notes

- Car pose is published from measured Gazebo model state, not from direct Gazebo parameter lookups.
- PX4 is intentionally bootstrapped into `external/PX4-Autopilot` instead of being committed to git.
- See `ANALYSIS.md` for design tradeoffs and known weaknesses.

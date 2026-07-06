# ANALYSIS

Answers are grounded in the system implemented in this repository. Where the stack is incomplete or fragile, that is stated directly.

## Q1 - The Conflict

Before writing code, I would ask the AI team: what is the end-to-end latency distribution (median and 99th percentile), whether timestamps represent capture time or publish time, what frame the detections are expressed in, whether updates can be guaranteed at a minimum rate during occlusion, and whether they can provide a short-horizon velocity estimate with the pose. I would ask the control team: what control period they actually need, how much position noise causes oscillation, whether they can accept predicted/interpolated targets between detections, and what maximum tracking error is acceptable during brief dropouts.

If AI latency stays near 150 ms and control truly needs 50 ms updates, the architecture cannot be a naive relay from `/car/position` to `/drone/waypoint`. I would insert a tracker/predictor node that timestamps every measurement, maintains a filtered state, extrapolates to "now" at 20 Hz, and publishes bounded commands. If control can tolerate prediction, I keep a single follower output topic and hide the estimator internally. If they cannot, I split responsibilities: AI publishes delayed observations, the estimator publishes short-horizon predictions, and control consumes predictions while health logic monitors innovation/residuals. If AI can reduce latency below one control period, the design simplifies back toward direct forwarding with outlier rejection only.

## Q2 - The Bug

If I found a wrong coordinate frame mid-project, I would not fix it silently. I would fix it immediately in code, add a short note in the PR/commit message, and tell the control and integration owners the same day with one concrete symptom ("drone tracks in the wrong axis") and the frame mapping change. Silent fixes are how demo-day regressions happen, especially in ROS/PX4 systems where `map`, ENU, and NED look "close enough" until motion starts.

Two hours before a demo, I would still fix it, but I would not expand scope. I would apply the minimal frame conversion, rerun one recorded integration test, and show one plot proving the drone and car paths align. I would not refactor unrelated nodes or re-tune gains in the same change window. If the fix touched shared interfaces, I would notify the team before the demo, not after. This project hit exactly that bug: ROS `map`/ENU waypoints were forwarded into PX4 NED setpoints without conversion, so the drone took off but did not follow correctly until north/east were swapped in `px4_manager`.

## Q3 - Your Weaknesses

1. Follow tracking error is still large. In run `artifacts/run_20260704T100424Z/plots/summary.txt`, mean horizontal waypoint error was 8.41 m with a peak of 17.27 m, even though the commanded follow offset is 6 m. That means the stack is operationally "following," but not tightly.

2. PX4 bring-up is brittle. Arming can fail on the first attempts if offboard/pre-arm checks are not ready, and stale `px4`/`gzserver` processes can block the next launch. The system recovers with retries and cleanup, but it is not yet one-shot reliable on a clean machine.

3. Simulation timing jitter is measurable. The same run logged an RTF warning at 0.78 before recovering to 1.01, and takeoff plus follow mode took about 14 s after connection. Under load, that shows up as delayed follow engagement and higher tracking error.

## Q4 - One More Week

I would add a dedicated state-estimation/prediction layer between `/car/position` and `/drone/waypoint`, with explicit timestamping, outlier rejection, and constant-velocity prediction to the control tick. That gives the largest reliability gain because it addresses the core conflict from Q1 and the largest measured weakness from Q3 in one place. Alternatives like better meshes, more logging, or tighter PX4 gains help, but they do not fix stale or mis-timed target updates. Prediction plus timestamp-aware buffering would reduce oscillation, make timeout handling smarter, and give measurable improvement in mean tracking error without changing the external topic contract.

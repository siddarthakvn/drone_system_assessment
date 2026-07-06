# Submission Checklist

## Before Submitting

1. Push the repository to a public GitHub repo.
2. Confirm a clean machine can run:
   - `bash scripts/bootstrap_px4.sh`
   - `python3 -m colcon build --packages-select drone_system`
   - `ros2 launch drone_system full_stack.launch.py`
3. Confirm CI workflow exists at `.github/workflows/integration_test.yml`.
4. Confirm `ANALYSIS.md` is filled in.
5. Attach or reference one sample run under `artifacts/run_*` only if you want example outputs in the repo; otherwise keep `artifacts/` gitignored.

## Email Template

Send to: `info@invictron.in`

Subject: Robotics System Engineer Assessment Submission - <Your Name>

Body:

```text
Hi,

Please find my assessment submission here:
<PUBLIC_GITHUB_REPO_URL>

Hardest part:
Integrating PX4 SITL, Gazebo Classic, and ROS 2 offboard control into one reproducible launch while keeping failure handling and telemetry artifacts intact. The most difficult debugging issue was a coordinate-frame mismatch between ROS map/ENU waypoints and PX4 NED setpoints, which made the drone take off correctly but track the wrong horizontal target until the conversion was fixed in px4_manager.

Available for a 30-minute live review:
1. <Day, Date, Time, Timezone>
2. <Day, Date, Time, Timezone>

Thanks,
<Your Name>
```

Replace the review times before sending.

## Suggested Final Git Commands

```bash
git init
git add .
git commit -m "Complete drone follower assessment submission."
git branch -M main
git remote add origin <PUBLIC_GITHUB_REPO_URL>
git push -u origin main
```

Do not commit `external/PX4-Autopilot/` or local `artifacts/` unless you intentionally want example outputs in the public repo.

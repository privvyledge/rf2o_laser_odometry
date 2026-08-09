# rf2o_laser_odometry

ROS 2 `ament_cmake` package for RF2O laser odometry. RF2O estimates planar robot motion from consecutive 2D laser scans using range-flow constraints, dense scan gradients, and robust optimization instead of explicit point correspondences.

For the original algorithm, see **Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016**: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf

## What this version adds

This package keeps the original RF2O scan-matching core, but updates the ROS 2 integration and hardens the runtime behavior for robot use:

- **Composed system support**: ROS 2 component support through `rclcpp_components`, with the standalone `rf2o_laser_odometry_node` executable still available.
- **Robustness & Input Sanitization**: Automatically handles `NaN`, `inf`, and out-of-range values in laser scans. Invalid beams are sanitized to `0.f`, preventing solver crashes or "explosive" velocity estimates. Rejection of empty scans, scan width changes, invalid scan timing, and invalid scan field of view.
- **Accurate Timing**: Uses actual scan-to-scan timestamps (`dt`) for velocity calculation instead of assuming a fixed frequency. This significantly improves accuracy on systems with jittery scan rates.
- **Determinism**: Safer solver behavior when the RF2O normal equations or covariance calculations become non-finite or singular.
- **EKF-Ready Covariance**: Computes a proper $3 \times 3$ covariance matrix for the robot's `base_link` frame using a Jacobian/Adjoint transform from the laser frame.
- **Configurable covariance**: Configurable covariance publication for pose and twist, including optional RF2O-derived twist covariance.
- **Lateral Motion Support**: Full support for robots that can move laterally (`linear.y`), with correct twist transformation into the base frame.
- **Numerical Stability**: Uses rank-checked QR decomposition for solving the range flow equations, ensuring the node remains stable even in feature-poor environments (e.g., long corridors).
- **Performance Optimized**: Minimized data copies and used throttled logging to reduce CPU overhead and console noise.
- **Correct Twist Output Frame**: Robot-frame twist output with both `linear.x` and `linear.y`.
- **Proper TF handling**: TF startup handling that waits for the laser-to-base transform before initializing odometry.
- **Reduced verbosity**: Reduced high-rate log spam by moving per-scan odometry and timing logs to `DEBUG`.

Compared with the original ROS 2 port, this version is more configurable, better suited for composed systems, and more defensive around real-world laser scan data.

## Build

Run from the ROS 2 workspace root that contains this package:

```bash
colcon build --packages-select rf2o_laser_odometry --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Test

```bash
colcon test --packages-select rf2o_laser_odometry
colcon test-result --verbose
```

This package currently does not include automated tests. Runtime changes should be validated with representative scan data or a bag, especially when changing odometry math, TF behavior, or covariance settings.

## Run as a node

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

The default launch file subscribes to `/scan`, publishes odometry on `/odom_rf2o`, uses `odom` as the odometry frame, and uses `base_link` as the robot frame.

You can also run the executable directly:

```bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom_rf2o \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom
```

## Run as a component

The node is registered as:

```text
rf2o::CLaserOdometry2DNode
```

Example composition command:

```bash
ros2 component load /ComponentManager rf2o_laser_odometry rf2o::CLaserOdometry2DNode
```

Use the same ROS parameters listed below when loading the component into a container.

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `laser_scan_topic` | string | `/scan` | Laser scan input topic. |
| `odom_topic` | string | `/odom_rf2o` | Odometry output topic. |
| `base_frame_id` | string | `base_link` | Robot base frame used for odometry output. |
| `odom_frame_id` | string | `odom` | Odometry frame used for the published odometry and optional TF. |
| `publish_tf` | bool | `true` | Publish `odom_frame_id -> base_frame_id` TF. |
| `init_pose_from_topic` | string | `/base_pose_ground_truth` | Optional odometry topic used once for the initial pose. Set to an empty string to start from identity. |
| `freq` | double | `10.0` | Processing timer frequency in Hz. Must be greater than zero. |
| `publish_covariance` | bool | `true` | Fill odometry pose and twist covariance arrays. |
| `pose_covariance_diagonal` | double array, length 6 | `[0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.0025]` | Pose covariance diagonal for x, y, z, roll, pitch, yaw. |
| `twist_covariance_diagonal` | double array, length 6 | `[0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.01]` | Twist covariance diagonal for vx, vy, vz, vroll, vpitch, vyaw. |
| `use_rf2o_twist_covariance` | bool | `false` | Use the RF2O solver covariance for planar twist covariance when it is finite. |
| `covariance_scale` | double | `1.0` | Positive scale applied to RF2O-derived twist covariance. |
| `enable_zero_velocity_detection` | bool | `true` | Suppress the pose increment while consecutive scans indicate no motion. |
| `zero_velocity_linear_threshold` | double | `0.02` | Solved translational speed in m/s below which a scan pair counts as still. |
| `zero_velocity_angular_threshold` | double | `0.05` | Solved rotational speed in rad/s below which a scan pair counts as still. |
| `zero_velocity_scan_diff_threshold` | double | `0.03` | Mean absolute range difference in m between consecutive scans below which they count as still. Set to zero or less to disable this cross-check. |
| `zero_velocity_hold_scans` | int | `3` | Consecutive still scans required before the gate engages. |
| `zero_velocity_release_scans` | int | `2` | Consecutive moving scans required before the gate releases. |
| `zero_velocity_twist_topic` | string | `""` | Optional external velocity topic used only to confirm a scan-derived stationary decision. Empty disables it. |
| `zero_velocity_twist_type` | string | `auto` | Message type on that topic. `auto` takes it from the publisher. Accepts `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `geometry_msgs/msg/TwistWithCovarianceStamped` and `nav_msgs/msg/Odometry`. |
| `zero_velocity_twist_timeout` | double | `0.5` | Age in seconds after which the external signal is ignored. |

## Zero-velocity detection

A stationary scanner still produces a small scan-matching solution whose sign varies from scan to scan. Integrating it turns sensor noise into an unbounded random walk in the reported pose, which is most visible as yaw drift on a parked robot. The node therefore classifies each scan pair before integrating it, and holds the pose while the pair looks stationary.

Two independent cues must agree:

1. the solved translational and rotational speed for the pair, compared against `zero_velocity_linear_threshold` and `zero_velocity_angular_threshold`;
2. the mean absolute range difference between the two scans over the beams that are valid in both, compared against `zero_velocity_scan_diff_threshold`. This does not depend on the solver, so it also catches a solver that wrongly reports near-zero motion.

Both edges are held, by `zero_velocity_hold_scans` when engaging and `zero_velocity_release_scans` when releasing, so that neither a single quiet scan pair nor an isolated noise spike can chatter the gate.

While the gate is engaged the node keeps publishing at its normal rate: the pose is held at its last value and the twist is exactly zero. It never stops publishing, so a rate-based health check still sees a live source.

`zero_velocity_twist_topic` is optional and off by default. When set, the external velocity can only veto a stationary decision that the scans already produced; it can never declare the robot stationary on its own, and a missing or stale signal leaves detection scan-only. This keeps the node usable standalone and identical across robots.

## Topics and frames

Subscribed topics:

- `laser_scan_topic` (`sensor_msgs/msg/LaserScan`)
- `init_pose_from_topic` (`nav_msgs/msg/Odometry`), only when configured
- `zero_velocity_twist_topic` (`geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `geometry_msgs/msg/TwistWithCovarianceStamped` or `nav_msgs/msg/Odometry`), only when configured

Published topics:

- `odom_topic` (`nav_msgs/msg/Odometry`)

Published TF, when `publish_tf` is true:

- `odom_frame_id -> base_frame_id`

The node requires a TF transform from the scan frame to `base_frame_id` before the first scan can initialize RF2O.

## Notes for robot_localization

This version can publish covariance values directly in `nav_msgs/msg/Odometry`, which makes it easier to fuse RF2O with `robot_localization`. The default z, roll, and pitch covariances are intentionally large because RF2O estimates planar motion. Tune the x, y, yaw, vx, vy, and vyaw covariance values for your platform and scan environment.

When `use_rf2o_twist_covariance` is true, the RF2O planar covariance is transformed from the laser frame into the robot base frame and written into the vx, vy, and vyaw covariance block. The configured twist covariance diagonal remains a floor for those values.

## Current limitations

- RF2O assumes a fixed laser-to-base transform and planar motion.
- The scan width is fixed after initialization; scans with a different number of ranges are rejected.
- Automated tests are not included yet.

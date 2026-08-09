# Changelog

## [Unreleased]

### Added

- Added scan-derived zero-velocity detection. A stationary scanner still produces a small, sign-random scan-matching solution, and integrating it made the reported pose perform an unbounded random walk. The pose increment is now suppressed while consecutive scans indicate no motion, configured through:
  - `enable_zero_velocity_detection`
  - `zero_velocity_linear_threshold`
  - `zero_velocity_angular_threshold`
  - `zero_velocity_scan_diff_threshold`
  - `zero_velocity_hold_scans`
  - `zero_velocity_release_scans`
- Added an optional external velocity input through `zero_velocity_twist_topic`, `zero_velocity_twist_type` and `zero_velocity_twist_timeout`. It accepts `geometry_msgs/msg/Twist`, `geometry_msgs/msg/TwistStamped`, `geometry_msgs/msg/TwistWithCovarianceStamped` and `nav_msgs/msg/Odometry`, resolving the type from the publisher by default. It can only confirm a scan-derived stationary decision, never produce one, so the node stays self-contained and behaves identically across robots when unset.

### Changed

- While the zero-velocity gate is engaged the node keeps publishing at its normal rate with the pose held and the twist set to exactly zero, rather than going silent.

## [Released] - 2026-05-26

### Added

- Added ROS 2 component support with `rclcpp_components`, while keeping the standalone `rf2o_laser_odometry_node` executable.
- Added configurable odometry covariance publication through:
  - `publish_covariance`
  - `pose_covariance_diagonal`
  - `twist_covariance_diagonal`
  - `use_rf2o_twist_covariance`
  - `covariance_scale`
- Added robot-frame lateral velocity output in `odom.twist.twist.linear.y`.
- Added scan validation for empty scans, scan width changes, invalid timestamps, and invalid angular field of view.
- Added scan range sanitization for NaN, infinity, non-positive, below-minimum, and above-maximum range values.
- Added TF startup handling so the node waits for the laser-to-base transform instead of initializing with a missing transform.

### Changed

- Moved high-rate odometry and timing logs from `INFO` to `DEBUG`.
- Reworked processing to use a ROS timer, making the node suitable for standalone execution and composition.
- Kept the RF2O algorithm object as a plain C++ class with injected ROS logger/clock state, avoiding a nested ROS node in composed deployments.
- Updated package metadata to package format 3 and added component-related dependencies.
- Updated the scan matcher to reject non-finite or singular linear systems instead of propagating invalid odometry.
- Updated initial pose handling so the default orientation is the identity quaternion.
- Updated twist calculation to use the robot-frame pose delta, including lateral velocity.

### Fixed

- Fixed first-scan initialization so the old laser and robot poses are initialized from the actual starting pose.
- Fixed invalid scan values causing NaNs or infinities in pyramid construction, derivative computation, weighting, covariance calculation, and pose filtering.
- Fixed divide-by-zero hazards in pyramid filtering, derivative calculation, and weight normalization.
- Fixed width mismatch handling so malformed scans are rejected before reaching the RF2O solver.
- Fixed startup behavior when TF is temporarily unavailable.
- Fixed invalid `freq` handling so the processing timer falls back to 10 Hz.
- Fixed all-invalid scans so they are rejected instead of publishing unchanged odometry with a newer timestamp.
- Fixed rejected scan handling so failed solver updates do not replace the previous accepted scan reference.

### Notes

- The package has been locally built and tested by the maintainer with these changes.
- Recommended follow-up coverage includes scan sanitization, TF startup delay, covariance publication, scan width mismatch, and robot-frame twist output.

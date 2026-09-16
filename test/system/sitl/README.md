# SITL system tests

These tests exercise the complete deployed pipeline with PX4 SITL and the
simulated vehicle. Required scenarios are hover, a mission waypoint sequence,
stale state/reference, solver deadline recovery, mission completion and safe
landing.

Each run must retain a manifest, mission and parameter checksums, ROS logs,
PX4 ULog, rosbag, metrics and a machine-readable PASS/FAIL report outside the
source tree.

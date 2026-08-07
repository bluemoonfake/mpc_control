# Validated MPC prediction horizon.
# header.stamp is the horizon origin; point times are relative to that origin.
std_msgs/Header header
uint64 trajectory_id
uint64 source_sequence
TrajectoryPoint[] points

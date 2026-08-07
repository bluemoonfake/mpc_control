# Complete user/provider-supplied reference trajectory.
# header.stamp is the trajectory epoch; header.frame_id is the source frame.
std_msgs/Header header
uint64 trajectory_id
bool hold_after_end
TrajectoryPoint[] points

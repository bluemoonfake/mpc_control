# Relative step command expressed in the controller's world ENU frame.
# Applying a step changes only the captured hold position; hold yaw is preserved.
std_msgs/Header header
float64[3] delta_position
bool valid

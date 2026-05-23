# MINCO trajectory for MPC controller
# Waypoints + segment times + boundary state for trajectory reconstruction

std_msgs/Header header

# Trajectory identity assigned by the active trajectory owner
uint64 trajectory_id

# Goal identity derived from the final target of this trajectory
uint64 goal_id

# Time origin used by downstream samplers for the full active trajectory
builtin_interfaces/Time start_time

# Waypoints (N+1 points for N segments)
geometry_msgs/Point[] waypoints

# Segment durations (N values, one per segment)
float64[] segment_times

# Initial velocity at first waypoint
geometry_msgs/Vector3 initial_velocity

# Initial acceleration at first waypoint
geometry_msgs/Vector3 initial_acceleration

# Terminal boundary used to rebuild rolling local trajectories.
# terminal_stop=true means this trajectory ends at a real stop goal.
bool terminal_stop
geometry_msgs/Vector3 terminal_velocity
geometry_msgs/Vector3 terminal_acceleration

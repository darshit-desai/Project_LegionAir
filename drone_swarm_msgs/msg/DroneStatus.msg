# DroneStatus.msg
# Status of the DroneStatus

std_msgs/Int32 id # ID of the Drone
std_msgs/Int32 neighbor_left # Left neighbor of the drone
std_msgs/Int32 neighbor_right # Right neighbor of the drone
std_msgs/Float64 radius # Radius in polar coordiantes
std_msgs/Float64 phase_angle # Phase angle in polar coordinates, deg
geometry_msgs/Pose pose # Pose of the drone
std_msgs/Bool anchor # Anchor bool flag
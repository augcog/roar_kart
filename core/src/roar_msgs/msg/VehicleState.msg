std_msgs/Header header

# raw sensor data
nav_msgs/Odometry odometry
sensor_msgs/Imu imu

VehicleStatus vehicle_status

# global planner
nav_msgs/Path global_path 

# behavior planner 
BehaviorStatus behavior_status

# local planner
geometry_msgs/PoseStamped next_waypoint


# Vehicle control commands
VehicleControl vehicle_control
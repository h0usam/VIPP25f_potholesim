std_msgs/Header header

geometry_msgs/Point center    # world frame (x, y, z)
float32 a                     # semi-major axis [m]
float32 b                     # semi-minor axis [m]
float32 yaw                   # radians, orientation wrt x-axis of road
float32 depth_max             # maximum depth [m]
float32 edge_angle            # slope angle of edge [rad]
float32 severity              # 0..1 or any scale you choose

int32 id                      # pothole identifier

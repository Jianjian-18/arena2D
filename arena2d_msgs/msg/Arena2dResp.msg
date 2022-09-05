# arena2d response message.

sensor_msgs/LaserScan observation
# distancee and angle to goal, angle is in degree
float32[2] goal_info
# coordinate of goal
float32[2] goal_pos
# position of all robot obstacle, we assume there are maximal 20/2 = 100 obstacle
float32[200] robot_obstacle_pos
# position of all human obstacle, we assume there are maximal 20/2 = 100 obstacle
float32[200] human_obstacle_pos
# robot coordinate and orientation, theta is in radian
geometry_msgs/Pose2D robot_pos
float32 reward
bool  done
float32 mean_reward
float32 mean_success
Arena2dContAction last_action
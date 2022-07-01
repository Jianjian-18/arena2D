# arena2d response message.

sensor_msgs/LaserScan observation
# distancee and angle to goal, angle is in degree
float32[2] goal_pos
# coordinate of goal
float32[2] goal_xy
# position of all dynamic obstacle, we assume there are maximal 26/2 = 13 obstacle
float32[26] obstacle_pos
# robot coordinate and orientation, theta is in radian
geometry_msgs/Pose2D robot_pos
float32 reward
bool  done
float32 mean_reward
float32 mean_success
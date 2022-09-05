#!/usr/bin/env python
import rospy
from arena2d_msgs.msg import Arena2dResp,Arena2dContAction
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Pose, Pose2D, Point, Vector3, Quaternion
import argparse
import time
import tf
import tf.transformations as tft
import numpy as np
from collections import deque
import threading
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA, String


class IntermediateRosNode:
    """
        rviz doesn't support show custom message, we need to either split the
        custom message to standard message or write rviz plug,the former is much
        easier.
    """

    def __init__(self, idx_env=0, laser_scan_publish_rate: int = 0):
        """
        Args:
            idx_env: the environment index we want to visualze by rviz. Defaults to 0
            laser_scan_publish_rate (int, optional): [the pushlish rate to the laser scan, if the it is set to 0
            then the publishment of the laser scan will synchronized with receiving message]. Defaults to 0.
        """
        self._robot_frame_id = "arena_robot_{:02d}".format(idx_env)
        self._header_seq_id = 0
        rospy.init_node("arena_env{:02d}_redirecter".format(idx_env), anonymous=True)
        self._idx_env = idx_env

        # ⭐hyperparameter
        self._map_origin = [0,0]
        self.points_idx = 0
        self.pre_goal = [0.0,0.0]
        self.goal = Marker()
        self.line_strip = MarkerArray()
        self.obstacle_group = MarkerArray()
        self.marker_max = 500
        
        
        if laser_scan_publish_rate == 0:
            # a flag to show where the laser scan is publised in a asynchronized way
            self._is_laser_scan_publish_asyn = False
        else:
            self._is_laser_scan_publish_asyn = True
            # set a cache and lock for accessing it in multi-threading env
            # we set the maxlen to three, since we want to
            # give more info about current status
            self._laser_scan_cache = deque(maxlen=3)
            self._laser_scan_cache_lock = threading.Lock()
            self._laser_scan_pub_rate = laser_scan_publish_rate
            self._laser_scan_pub_rater = rospy.Rate(hz=laser_scan_publish_rate)
            self._new_laser_scan_received = False
            self.start_time = rospy.Time.now()
            
        self._setSubPub()

    def _setSubPub(self, laser_scan_publish_rate: int = 0):
        namespace_sub = "arena2d/env_{:d}/".format(self._idx_env)
        namespace_pub = "arena2d_intermediate/"
        namespace_standard = "standard_arena2d/"
        # publisher
        self._pub_laser_scan = rospy.Publisher(namespace_pub + "laserscan", LaserScan, queue_size=1, tcp_nodelay=True)
        # self._pub_path = rospy.Publisher(namespace_pub + "path", Path, queue_size=10, tcp_nodelay=True)
        self._pub_goal = rospy.Publisher(namespace_pub + "goal", Marker, queue_size=1, tcp_nodelay=True)
        self._pub_robot = rospy.Publisher(namespace_pub + "robot", Marker, queue_size=1, tcp_nodelay=True)
        self._pub_path = rospy.Publisher(namespace_pub + "path", MarkerArray, queue_size=1, tcp_nodelay=True)
        self._pub_obstacle = rospy.Publisher(namespace_pub + "obstacle", MarkerArray, queue_size=1, tcp_nodelay=True)
        # transform broadcaseter for robot position
        self._tf_rospos = tf.TransformBroadcaster()
        rospy.loginfo("intermediate node is waiting for connecting env[{:02d}]".format(self._idx_env))

        self._standard_laser_scan = rospy.Publisher(namespace_standard + "laserscan", LaserScan, queue_size=1, tcp_nodelay=True)
        self._standrad_odometry = rospy.Publisher(namespace_standard + "odometry", Odometry, queue_size=1, tcp_nodelay=True)
        times = 0
        # subscriber
        # According to the testing,enable tcp_nodelay can double the performance


        self._sub_map = rospy.Subscriber("map", OccupancyGrid, self._mapCallback ,tcp_nodelay=True)        

        self._sub = rospy.Subscriber(namespace_sub + "response", Arena2dResp,
                                     self._arena2dRespCallback, tcp_nodelay=True)

                                     
        # # give rospy enough time to establish the connection, without this procedure, the message to
        # # be published at the beginning could be lost.
        while self._sub.get_num_connections() == 0:
            time.sleep(0.1)
            times += 1
        rospy.loginfo("Successfully connected with arena-2d simulator, took {:3.1f}s.".format(.1 * times))
    def _mapCallback(self, msg: OccupancyGrid):

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.info)
        self._map_origin[0] = msg.info.width / 2
        self._map_origin[1] = msg.info.height / 2
        self._map_broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "map"
        static_transformStamped.transform.translation.x = - (self._map_origin[0] * msg.info.resolution)
        static_transformStamped.transform.translation.y = - (self._map_origin[1] * msg.info.resolution)
        static_transformStamped.transform.translation.z = 0
        static_transformStamped.transform.rotation.x = 0
        static_transformStamped.transform.rotation.y = 0
        static_transformStamped.transform.rotation.z = 0
        static_transformStamped.transform.rotation.w = 1  
        self._map_broadcaster.sendTransform(static_transformStamped)    
        # print(static_transformStamped.transform.translation.x, static_transformStamped.transform.translation.y,"\n")
    def _show_scan_in_rviz(self, scan: LaserScan):
        laser_scan = scan
        laser_scan.angle_min = 0
        laser_scan.angle_max = 2 * np.pi
        self.num_beam = rospy.get_param("/arena_sim/settings/observation_space_num_beam")
        laser_scan.angle_increment = np.pi/(self.num_beam/2)
        laser_scan.range_min = 0
        # not sure about it.
        laser_scan.range_max = 50   
        # set up header
        laser_scan.header.frame_id = self._robot_frame_id
        laser_scan.header.seq = self._header_seq_id
        laser_scan.header.stamp = rospy.Time.now()

        self._header_seq_id += 1

        if not self._is_laser_scan_publish_asyn:
            self._pub_laser_scan.publish(laser_scan)
        else:
            with self._laser_scan_cache_lock:
                self._laser_scan_cache.append(laser_scan)    
    # ⭐ also reset path    
    def _show_goal_robot_in_rviz(self, x, y, robot_pos: Pose2D):     
        # if((self.pre_goal[0] != x) & (self.pre_goal[1] != y)):
        #     for i in self.line_strip.markers:
        #         i.action = Marker.DELETEALL   
        #     for i in self.obstacle_group.markers:
        #         i.action = Marker.DELETEALL   
        self.goal.header.frame_id = "world"
        self.goal.id = 0
        self.goal.type = self.goal.SPHERE
        self.goal.action = self.goal.ADD
        self.goal.pose = Pose(Point(x,y,0), Quaternion(0, 0, 0, 1))
        self.goal.color.r = 1.0
        self.goal.color.g = 1.0
        self.goal.color.b = 0.0
        self.goal.color.a = 1.0
        self.goal.scale.x = 0.2
        self.goal.scale.y = 0.2
        self.goal.scale.z = 0.2
        self.goal.frame_locked = False
        self._pub_goal.publish(self.goal)

        robot = Marker()
        robot.header.frame_id = "world"
        robot.id = 1
        robot.type = robot.SPHERE
        robot.action = robot.ADD
        robot.pose = Pose(Point(robot_pos.x,robot_pos.y,0), Quaternion(0, 0, 0, 1))
        robot.color.r = 0.1
        robot.color.g = 0.1
        robot.color.b = 1.0
        robot.color.a = 1.0
        robot.scale.x = 0.2
        robot.scale.y = 0.2
        robot.scale.z = 0.2
        robot.frame_locked = False
        self._pub_robot.publish(robot)

    def _show_path_in_rviz(self,robot_pos: Pose2D):
        marker = Marker()
        marker.header.frame_id = "world"
        # marker.id = self.points_idx
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose = Pose(Point(robot_pos.x,robot_pos.y,0), Quaternion(0, 0, 0, 1))
        marker.color.r = 0.8
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.frame_locked = False
        self.points_idx+=1
        if(self.points_idx > self.marker_max):
           self.line_strip.markers.pop(0)
        self.line_strip.markers.append(marker) 
        id = 0
        for m in self.line_strip.markers:
            m.id = id
            id += 1
        self._pub_path.publish(self.line_strip)
        # rospy.loginfo('msg published')
    def _show_obstacle_in_rviz(self,robot_obstacle_pos, human_obstacle_pos):
        step = 2
        robot_tmp = [robot_obstacle_pos[i:i+step] for i in range(0,len(robot_obstacle_pos),step)]
        human_tmp = [human_obstacle_pos[i:i+step] for i in range(0,len(human_obstacle_pos),step)]
        count_robot_obstacle = 0;
        for j,i in enumerate(robot_tmp):
            if (i[0] or i[1]):
                obstacle = Marker()
                obstacle.header.frame_id = "world"
                obstacle.id = j
                obstacle.type = obstacle.SPHERE
                obstacle.action = obstacle.ADD
                obstacle.pose = Pose(Point(i[0],i[1],0), Quaternion(0, 0, 0, 1))
                obstacle.color.r = 0.6
                obstacle.color.g = 1.0
                obstacle.color.b = 0.5
                obstacle.color.a = 1.0
                obstacle.scale.x = 0.2
                obstacle.scale.y = 0.2
                obstacle.scale.z = 0.2
                obstacle.frame_locked = False
                for b,a in enumerate(self.obstacle_group.markers):
                    if (self.obstacle_group.markers[b].id == obstacle.id):
                        self.obstacle_group.markers.pop(b)
                self.obstacle_group.markers.append(obstacle) 
                count_robot_obstacle+=1     
        for k,l in enumerate(human_tmp):
            if (l[0] or l[1]):
                obstacle = Marker()
                obstacle.header.frame_id = "world"
                obstacle.id = k + count_robot_obstacle
                obstacle.type = obstacle.SPHERE
                obstacle.action = obstacle.ADD
                obstacle.pose = Pose(Point(l[0],l[1],0), Quaternion(0, 0, 0, 1))
                obstacle.color.r = 1.0
                obstacle.color.g = 0.6
                obstacle.color.b = 0.5
                obstacle.color.a = 1.0
                obstacle.scale.x = 0.2
                obstacle.scale.y = 0.2
                obstacle.scale.z = 0.2
                obstacle.frame_locked = False
                for b,a in enumerate(self.obstacle_group.markers):
                    if (self.obstacle_group.markers[b].id == obstacle.id):
                        self.obstacle_group.markers.pop(b)
                self.obstacle_group.markers.append(obstacle)  
        self._pub_obstacle.publish(self.obstacle_group)        

    def _show_odometry_in_rviz(self,last_action: Arena2dContAction, robot_pos: Pose2D):
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.header.seq = self._header_seq_id
        odom.header.stamp = rospy.Time.now()

        # self._header_seq_id += 1

        odom.child_frame_id = self._robot_frame_id
        odom.pose.pose.position.x = robot_pos.x;
        odom.pose.pose.position.y = robot_pos.y;
        odom.pose.pose.position.z = 0.0;
        q = tf.transformations.quaternion_from_euler(0, 0, robot_pos.theta + np.pi/4)
        odom.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        odom.twist.twist.linear.x = last_action.linear;
        odom.twist.twist.linear.y = 0.0;
        # not holonomic
        odom.twist.twist.angular.z = last_action.angular;
        self._standrad_odometry.publish(odom)
  
    def _arena2dRespCallback(self, resp: Arena2dResp):
        curr_time = rospy.Time.now()
        robot_pos = resp.robot_pos
        self._tf_rospos.sendTransform((robot_pos.x, robot_pos.y, 0),
                                      tft.quaternion_from_euler(0, 0, robot_pos.theta),
                                      curr_time,
                                      self._robot_frame_id, "world")      

        self._show_scan_in_rviz(resp.observation)                                        
        self. _show_goal_robot_in_rviz(resp.goal_pos[0],resp.goal_pos[1], robot_pos)
        self._show_path_in_rviz(resp.robot_pos)
        self._show_obstacle_in_rviz(resp.robot_obstacle_pos, resp.human_obstacle_pos)

        self._standard_laser_scan.publish(resp.observation)
        self._show_odometry_in_rviz(resp.last_action, resp.robot_pos);
    def run(self):
        while not rospy.is_shutdown():
            if self._is_laser_scan_publish_asyn:
                with self._laser_scan_cache_lock:
                    len_cache = len(self._laser_scan_cache)
                    # if no cache, do nothing
                    # if cache size == 2 , laser scan pubish rate too slow compared to coming data
                    if len_cache == 0:
                        continue
                    else:
                        latest_laser_scan = self._laser_scan_cache[-1]
                        self._pub_laser_scan.publish(latest_laser_scan)
                        # it means the the interact rate is too high, some of the data will be discarded
                        if len_cache == 3 and rospy.Time.now().to_sec()-self.start_time.to_sec() > 10:
                            interact_rate = 2*1 / \
                                (self._laser_scan_cache[-1].header.stamp.to_sec() -
                                 self._laser_scan_cache[0].header.stamp.to_sec())
                            rospy.logwarn_once("The rate [{:3.1f} FPS] of republishment of the laser scan is lower compared to the \
                            receivings [approximately {:3.1f} FPS], therefore some the them are discareded".format(self._laser_scan_pub_rate, interact_rate))
                        # chane the cache size to 1
                        while len(self._laser_scan_cache) != 1:
                            self._laser_scan_cache.popleft()
                self._laser_scan_pub_rater.sleep()
            else:
                rospy.spin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=int, default=0,
                        help="the index of the environment whose response message need to be redicted!")
    parser.add_argument("--laser_scan_pub_rate", type=int, default=0,
                        help="set up the publishing rate of the laser scan, if it is set to 0, then the rate is synchronized with\
                        the receiving rate")
    args, unknown = parser.parse_known_args()
    helper_node = IntermediateRosNode(args.env, args.laser_scan_pub_rate)
    helper_node.run()
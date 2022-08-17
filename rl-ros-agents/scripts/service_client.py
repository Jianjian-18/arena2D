#!/usr/bin/env python
# -*- coding: utf-8
import rospy
import sys
from arena2d_msgs.srv import *
import numpy as np
import os
import yaml
import math
import random
import rospy
from geometry_msgs.msg import PoseStamped, Pose2D, Point
from std_srvs.srv import Empty

import argparse
rospy.init_node('service_client')  # 节点初始化
print("wait for service.")
ns = 'arena2d/'

service_name = "";

flag = 1;

if  (flag == 1):
    service_name = f"{ns}delete_model"
elif(flag == 2):
    service_name = f"{ns}move_model"
elif(flag == 3):
    service_name = f"{ns}spawn_model"
elif(flag == 4):
    service_name = f"{ns}spawn_pedestrian"
elif(flag == 5):
    service_name = f"{ns}pause"
elif(flag == 6):
    service_name = f"{ns}unpause"
rospy.wait_for_service(service_name)  # 等待服务端声明这个服务
print(f"{service_name} has started")

# 声明服务的本地代理，需要指定服务的名称（‘word_count’）和类型（WordCount）
# 这允许我们像使用本地函数一样使用服务。
service_client = rospy.ServiceProxy(service_name, DeleteModel)
# service_client = rospy.ServiceProxy(service_name, MoveModel)
# service_client = rospy.ServiceProxy(service_name, SpawnModel)
# service_client = rospy.ServiceProxy(service_name, SpawnPeds)
# service_client = rospy.ServiceProxy(service_name, Empty)
print("get service")

pose = Pose2D()
pose.x = -0.5 
pose.y = 1
pose.theta = 0.0

args_static = [1.0, 2.0, 0.0, 0.0]
args_dynamic = [1.0, 1.0, 0.5, 120.0]

waypoints_1 = [
                
                    Point(
                    -0,
                    2.5,
                    0)
                ,
                
                    Point(
                    -0.5,
                    2.5,
                    0)
                ,       
                    Point(
                    1,
                    3,
                    0)
            ]

waypoints_2 = [
                
                    Point(
                    -0.5,
                    2.5,
                    0)
                ,
                
                    Point(
                    0.7,
                    2.5,
                    0)
                ,
                
                    Point(
                    0,
                    2,
                    0)                
            ]

output= service_client("all") # delete
# output= service_client("robot",pose) # move
# output= service_client("static",*args_static) # spawn static
# output= service_client("dynamic",*args_dynamic) # spawn static
# output= service_client(waypoints_1) # spawn peds1
# output= service_client(waypoints_2) # spawn peds2
# output = service_client()
print(output)
# rospy.spin()


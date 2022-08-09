#!/usr/bin/env python
# -*- coding: utf-8
import rospy
from arena2d_msgs.srv import *

def count_words(request):
    # 回调函数只接受WordCountRequest 类型的的参数，并返回一个WordCountResponse类型的值

    return DeleteModelResponse(True,"test server")

def server():
    rospy.init_node('service_server')  # 节点初始化
    # 声明服务，服务名称 'word_count',srv类型 WordCount，回调函数 count_words.
    service = rospy.Service('delete_model', DeleteModel, count_words) 
    print("Ready to service.")
    rospy.spin()
if __name__ == "__main__":
    server()    

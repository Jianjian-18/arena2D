from std_msgs.msg import String
import gym
import rospy
from geometry_msgs.msg import Twist
from arena2d_msgs.msg import RosAgentReq, Arena2dResp
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.monitor import Monitor
import numpy as np
import threading
from typing import Union, List, Tuple
import time
from gym import spaces
import os
from rosparam import upload_params
from yaml import load, safe_load
import rospkg
from std_srvs.srv import Empty, EmptyResponse
import sys
import math
# namespace of arena settings
NS_SETTING = "/arena_sim/settings/"
HOLONOMIC = "/is_holonomic/"
CONTINUOUS_ACTION = "/actions/continuous/"
DISCRETE_ACTION = "/actions/discrete/"
# OBSERVATION = "/plugins/"
robot_model = rospy.get_param('model')
robot_mode = rospy.get_param('mode')

# ⭐get robot action space

# DEFAULT_OBSERVATION_SPACE = os.path.join(
#     rospkg.RosPack().get_path("arena2d"),
#     "configs",
#     "observation_space",
#     f"{robot_model}.model.yaml",
# )

DEFAULT_SETTING = os.path.join(
    rospkg.RosPack().get_path("arena2d"),
    "settings.st",
)
# ⭐get robot observation space

def get_arena_envs(use_monitor=True, log_dir=None):
    # the num_envs should be set in the filt settings.st under the  folder of arena2d-sim
    num_envs = rospy.get_param(NS_SETTING + "num_envs")
    if log_dir is None:
        logs_file_names = [None] * num_envs
    else:
        logs_file_names = [os.path.join(log_dir, f"arena_env_{i}") for i in range(num_envs)]
    if use_monitor:
        return SubprocVecEnv([lambda i=i: Monitor(Arena2dEnvWrapper(i), logs_file_names[i]) for i in range(num_envs)])
    return SubprocVecEnv([lambda i=i: Arena2dEnvWrapper(i) for i in range(num_envs)])




class Arena2dEnvWrapper(gym.Env):
    def __init__(self, idx_env, _is_action_space_discrete = robot_mode):
        super().__init__()
        self._idx_env = idx_env
        print("\nRobot model is: {}, robot model is: {} \n".format(robot_model,robot_mode))
        if _is_action_space_discrete == 'discrete':
            self._is_action_space_discrete = True
        elif _is_action_space_discrete == 'continuous':
            self._is_action_space_discrete = False
        else: 
            print(f'invalid mode\n')
            return
        # ⭐选择mode
        
        self._action_discrete_list = ["move_forward", "move_backward", "turn_left", "turn_right", "turn_strong_left", "turn_strong_right", "stop"]
        self._action_discrete_map = rospy.get_param(DISCRETE_ACTION)
        # ⭐加载discrete action space

        rospy.init_node("arena_ros_agent_env_{:02d}".format(idx_env), anonymous=True, log_level=rospy.INFO)
        self._setSubPub()

        # ⭐comment for separate test: arena2d self don't have task generator

        # we use this to let main thread know the response is received which is done by another thread
        self.response_con = threading.Condition()
        self.resp_received = False

        # the following variables will be set on invoking the _arena2dRespCallback
        self.obs = None
        self.reward = None
        self.done = None
        self.info = None
        self._set_action_oberservation_space()
        self._setService()
        self.staged_mode = True
        if(not rospy.get_param("stage/stage")):
            self.staged_mode = False
        

    def _set_action_oberservation_space(self):
        action_space_linear_range = rospy.get_param(CONTINUOUS_ACTION + "linear_range")
        action_space_angular_range = rospy.get_param(CONTINUOUS_ACTION + "angular_range")

        lower_limit = np.array([action_space_linear_range[0], action_space_angular_range[0]], dtype=float)
        upper_limit = np.array([action_space_linear_range[1], action_space_angular_range[1]], dtype=float)

        num_beam = rospy.get_param(NS_SETTING + "observation_space_num_beam")    
        obervation_space_upper_limit = rospy.get_param(NS_SETTING + "observation_space_upper_limit")
        if not self._is_action_space_discrete:
            self.action_space = spaces.Box(low=lower_limit,
                                           high=upper_limit, dtype=float)
                                        #    high=upper_limit * 3, dtype=np.float)                                           
        # ⭐why hier *3 
        else:
            self.action_space = spaces.Discrete(len(self._action_discrete_list))
            
        self.observation_space = spaces.Box(low=0, high=obervation_space_upper_limit,
                                            shape=(num_beam+2,),dtype=float)
        # self.observation_space = spaces.Box(low=0, high=obervation_space_upper_limit,
        #                                     shape=(1, num_beam+2), dtype=np.float)                                            
        # ⭐最终将action和observation space 导入
        
    def step(self, action):
        rospy.logdebug("step in")
        self._pubRosAgentReq(action, env_reset=False)
        # get the observations from the arena simulater
        with self.response_con:
            while not self.resp_received:
                self.response_con.wait(1)
                if not self.resp_received:
                    rospy.logwarn(
                        "Environement wrapper [{}] didn't get the feedback within 1s \
                             from arena simulator after sending action".format(self._idx_env))
                    break
            self.resp_received = False
        rospy.logdebug("step out")
        return self.obs, self.reward, self.done, self.info

    def reset(self):
        self._pubRosAgentReq(env_reset=True)
        error_showed = False
        with self.response_con:
            while not self.resp_received:
                self.response_con.wait(1.5)
                if not self.resp_received:
                    rospy.logwarn(
                        "Environement wrapper [{}] didn't get the feedback within 1.5s from arena \
                            simulator after sending reset command".format(self._idx_env))
                    break
            self.resp_received = False            
        return self.obs

    def close(self):
        rospy.loginfo(f"env[{self._idx_env}] closed")
        self._pubRosAgentReq(env_close=True)

    def _setSubPub(self):
        namespace = "arena2d/env_{:d}/".format(self._idx_env)

        # publisher
        self._ros_agent_pub = rospy.Publisher(namespace + "request", RosAgentReq, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("env[{:d}] wrapper waiting for arena-2d simulator to connect!".format(self._idx_env))
        times = 0
        # subscriber
        # According to the testing,enable tcp_nodelay can double the performance
        self._arena2d_sub = rospy.Subscriber(namespace + "response", Arena2dResp,
                                             self._arena2dRespCallback, tcp_nodelay=True)
        # # give rospy enough time to establish the connection, without this procedure, the message to
        # # be published at the beginning could be lost.
        while self._ros_agent_pub.get_num_connections() == 0 or self._arena2d_sub.get_num_connections() == 0:
            time.sleep(0.1)
            times += 1
        rospy.loginfo("env[{:d}] connected with arena-2d simulator, took {:3.1f}s.".format(self._idx_env, .1 * times))

    def _setService(self):
        service_name = "task_generator"
        rospy.wait_for_service(service_name)
        print(f"{service_name} has started")
        self.service_client = rospy.ServiceProxy(service_name, Empty)

    def _pubRosAgentReq(self,
                        action: Union[List, Tuple, RosAgentReq] = None,
                        env_reset: bool = False,
                        env_close: bool = False):
        req_msg = RosAgentReq()

        if env_close:
            req_msg.arena2d_sim_close = True
        # reset environment
        elif env_reset:
            req_msg.env_reset = True
            # ⭐comment for separate test: arena2d self don't have task generator
        else:
            req_msg.env_reset = False
            if not self._is_action_space_discrete:
                assert isinstance(action, (list, tuple, np.ndarray)) and len(
                    action) == 2, "Type of action must be one of (list, tuple, numpy.ndarray) and \
                        length is equal to 2, current type of action is '{:4d}' ".format(type(action))

                req_msg.action.linear = action[0]
                req_msg.action.angular = action[1]
            else:
                action_name = self._action_discrete_list[action]

                # print("value of action is {}, action name is {}\n".format(action, action_name))

                for i in range(len(self._action_discrete_map)):
                    if self._action_discrete_map[i]['name'] == action_name:
                       req_msg.action.linear = self._action_discrete_map[i]['linear']
                       req_msg.action.angular = self._action_discrete_map[i]['angular']
                # req_msg.action.linear = self._action_discrete_map[action_name][0]
                # req_msg.action.angular = self._action_discrete_map[action_name][1]
        self._ros_agent_pub.publish(req_msg)
        rospy.logdebug("send action")
        if env_reset & (not self.staged_mode):
            self.service_client()

    def _arena2dRespCallback(self, resp: Arena2dResp):
        rospy.logdebug("received response")
        with self.response_con:
            obs = resp.observation.ranges
            goal_distance_angle = resp.goal_info
            # in current settings the observation not only contains laser scan but also
            # contains the relative distance and angle to goal position.
            # print(obs.shape)
            self.obs = np.array(obs + goal_distance_angle)
            # self.obs = np.array(obs + goal_distance_angle).reshape([1, -1])            
            # print("obs:"+obs.__str__()+" gda: "+goal_distance_angle.__str__())
            # print(" gda: "+goal_distance_angle.__str__())
            self.reward = resp.reward
            self.done = resp.done
            self.info = dict(mean_reward=resp.mean_reward, mean_success=resp.mean_success)
            self.resp_received = True
            self.response_con.notify()


if __name__ == "__main__":
    # comment out rospy.init_node in the class Arena2dEnv for the test!!!!!!!!!
    rospy.init_node("test")
    # def test_step(idx_env):
    #     env = Arena2dEnvWrapper(idx_env)
    #     action = [2, 0]
    #     _, reward, _, _ = env.step(action)
    #     # env.reset()
    #     print("env: {:d} reward {}".format(idx_env, reward))
    # for i in range(4):
    #     t = threading.Thread(target=test_step, args=(i,))
    #     t.start()

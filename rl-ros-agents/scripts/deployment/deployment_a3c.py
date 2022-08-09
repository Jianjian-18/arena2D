import rospy
from stable_baselines.common.vec_env import SubprocVecEnv
from rl_ros_agents.env_wappers.arena2dEnv import get_arena_envs, Arena2dEnvWrapper
from rl_ros_agents.utils.callbacks import SaveOnBestTrainingRewardCallback
from rl_ros_agents.utils import getTimeStr
from stable_baselines import A2C
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.evaluation import evaluate_policy
import tensorflow as tf
import random
import numpy as np
import os
import sys
import argparse


if __name__ == "__main__":

    envs = get_arena_envs()
    path_temp_model = os.path.join("results","A3C_TEMP")

        if os.path.exists(path_temp_model+".zip"):
            print("continue training the model...")
            model = A2C.load(path_temp_model,env=envs)
            reset_num_timesteps = False
        else:
            print("Can't load the model with the path: {}, please check again!".format(path_temp_model))
            envs.close()
            exit(-1)

        try:
            evaluate_policy(
                model=agent, env=env, n_eval_episodes=args.num_eps, deterministic=True
            )
        except StopReset:
            pass            
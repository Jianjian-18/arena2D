# rl-ros-agents(#TODO need a better name!)
## Introduction
rl-ros-agents is a package for training a local planner with the [stable-baseline](https://github.com/hill-a/stable-baselines) reinforcement learning approaches. It basically does the following two things:
    1. Defined the a environment wrapper which use ros messages to communicate with our arena simultor 
    2. Providing some handy scripts for training
![Working manner](/img/Working_manner_rl_ros_agent.png)
When the training get started, seveal instances of the environment wrapper will be created and each runs on a single process. The number of the environment and other parameters are loaded from the ROS parameter server, which are register by the arena simulator. Which means if number of the environments you want to use need to be changed, you need to modify the param **num_envs** in the file `settings.st` in the package *arena2d-sim*  .For each environment a pair of request message and response message will be created. For example if we create four environment, they may look like this:
```
/arena2d/env_0/request
/arena2d/env_0/response
/arena2d/env_1/request
/arena2d/env_1/response
/arena2d/env_2/request
/arena2d/env_2/response
/arena2d/env_3/request
/arena2d/env_3/response
```
The definitions of the request and response message can be found [here](../arena2d_msgs/msg). In each training step all environments will send the request in parallel and will be blocked util response messages are received. The main thread in the arena simulator will firstly collect the request messages and send the response messages back after finishing the updates.

## Prerequisites
   - Standard ROS setup. The link can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu) (currently we use ubuntu 20.04 and ros-noetic).
   - install conda and necessary dependencies:
   1. install conda see [here](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html)
   2. `$ sudo apt-get install cmake libsdl2-dev libfreetype-dev`

## Building
1. Create a catkin workspace:
    ```
    $ mkdir -p ~/ARENA2d_ws/src
    $ cd ~/ARENA2d_ws/
    ```
2. Clone this repository in the src-folder of your catkin workspace
 
    Compile the code and add the workspace to the ROS environment with:
    ```
    $ cd ~/ARENA2d_ws/src
    $ git clone https://github.com/zenghjian/arena2D.git
    $ cd ~/ARENA2d_ws/src/arena2D/rl-ros-agents
    $ conda env create -n arena2d python=3.8
    $ poetry shell && poetry install 
    $ pip install stable-baselines3[extra]
    $ cd ~/ARENA2d_ws
    $ pip install empy
    $ catkin_make -DUSE_ROS=ON
    $ source devel/setup.bash
    ```
    You may encounter some compilation problems when using `catkin_make`:
   ```
   $ /usr/bin/ld: /lib/x86_64-linux-gnu/libapr-1.so.0: undefined reference to `uuid_generate@UUID_1.0'
   ```
   to fix that, you can use following commands:
   ```
   $ ls ~/anaconda3/lib/libuuid*
   $ mkdir ~/anaconda3/libuuid
   $ mv ~/anaconda3/lib/libuuid* ~/anaconda3/libuuid
   $ catkin_make -DUSE_ROS=ON
   ```
   And
   
   ```
   $ ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'
   ```
   to fix that, you need find `PYTHONPATH` of catkin_pkg and add it to `./bashrc`:
   ```
   $ locate catkin_pkg
   # first line is path: /usr/lib/python3/dist-packages/catkin_pkg
   $ vim ~/.bashrc
   $ export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages # copy this in the bottom, then use wq! to quit
   $ source ~/.bashrc
   ```
	You can check if your pythonpath is correct by using `echo $PYTHONPATH`.

## Training

1. open a terminal and run the simulator:  
    ```
    cd ~/ARENA2d_ws
    source devel/setup.bash
    roslaunch arena2d arena2d_simulator.launch staged_mode:="true"
    ```

e.g `roslaunch arena2d arena_sim.launch model:=burger mode:=continuous ...`
| Program call      | Agent Flag   | Usage                                                          | Description                                         |
| ----------------- | ------------------------------- | -------------------------------------------------------------- | --------------------------------------------------- |
| `roslaunch arena2d arena_sim_video_on.launch ` | `model:=<model_name>`                       | burger, jackal, agvota, waffle_pi | Option of robot model       |
|                   | `mode:=<mode_name> `  | continuous, discrete   | Option of robot mobile mode                     |
|                   | `level_name:=<level_name>` | empty, random, svg, scenario | Option of task mode |
|                   | `level_mode:=<level_mode>` | --dynamic,   | Option of using dynamic obstacle (available mode need edit) |
|                   | `video_mode:=<video_mode>` | --disable-video, --enable-video | Option to use built-in video |
|                   | `scenerio_mode:=<scenerio_mode>` | true, false | Option to execute scenerio task |
|                   | `stage_mode:=<stage_mode>`       | true, false | Option to execute stage task |
2. open a new terminal and run the training script:
    ```
    cd ~/ARENA2d_ws
    source devel/setup.bash
    cd ~/ARENA2d_ws/src/arena2D/rl-ros-agents
    conda activate arena2d
    poetry shell
    python scripts/training/train_ppo.py
    ```

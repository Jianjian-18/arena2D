#include "RosNode.hpp"
RosNode::RosNode(Environment *envs, int num_envs, int argc, char **argv)
    : m_num_envs(num_envs), m_envs(envs)
{
    ROS_INFO("Ros node activated, waiting for connections from agents");
    m_env_connected = false;
    m_num_ros_agent_req_msgs_received = 0;
    m_env_close = 0;
    m_any_env_reset = false;
    m_envs_reset.resize(num_envs);

    m_actions_buffer = std::vector<std::unique_ptr<Twist>>();
    for (int i = 0; i < num_envs; i++)
    {
        m_actions_buffer.emplace_back(new Twist(0, 0));
    }

    /* init ros without sigint handler */
    ros::init(argc, argv, "arena_sim", ros::init_options::NoSigintHandler);
    m_nh_ptr = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("arena2d"));
    _publishParams();
    _setRosAgentsReqSub();
    _setArena2dResPub();
    _setRosService();
}
RosNode::~RosNode(){};

void RosNode::_setRosService()
{
    service_delete = m_nh_ptr->advertiseService(
        "delete_model", &RosNode::DeleteModelCallback, this);
    service_move = m_nh_ptr->advertiseService("move_model",
                                              &RosNode::MoveModelCallback, this);
    service_spawn = m_nh_ptr->advertiseService(
        "spawn_model", &RosNode::SpawnModelCallback, this);
    service_spawn_pedestrian = m_nh_ptr->advertiseService(
        "spawn_pedestrian", &RosNode::SpawnPedestrianCallback, this);
    service_pause =
        m_nh_ptr->advertiseService("pause", &RosNode::PauseCallback, this);
    service_unpause =
        m_nh_ptr->advertiseService("unpause", &RosNode::UnpauseCallback, this);
}
void RosNode::_publishParams()
{
    // namespace
    string ns = "~settings";
    ros::NodeHandle nh(ns);
    nh.setParam("num_envs", m_num_envs);
    nh.setParam("action_space_lower_limit",
                vector<float>{_SETTINGS->robot.backward_speed.linear,
                              _SETTINGS->robot.strong_right_speed.angular});
    nh.setParam("action_space_upper_limit",
                vector<float>{_SETTINGS->robot.forward_speed.linear,
                              _SETTINGS->robot.strong_left_speed.angular});
    nh.setParam("observation_space_num_beam", _SETTINGS->robot.laser_num_samples);
    nh.setParam("observation_space_upper_limit",
                _SETTINGS->robot.laser_max_distance);
}

void RosNode::_setRosAgentsReqSub()
{
    for (int i = 0; i < m_num_envs; i++)
    {
        std::stringstream ss;
        ss << "env_" << i << "/request";
        // set tcp no_delay, to get better performance.
        m_ros_agent_subs.push_back(m_nh_ptr->subscribe<arena2d_msgs::RosAgentReq>(
            ss.str(), 1, boost::bind(&RosNode::_RosAgentReqCallback, this, _1, i),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    }
}

void RosNode::_setTwistSub()
{
    for (int i = 0; i < m_num_envs; i++)
    {
        std::stringstream ss;
        ss << "env_" << i << "/request_standard";
        // set tcp no_delay, to get better performance.
        m_ros_agent_subs.push_back(m_nh_ptr->subscribe<geometry_msgs::Twist>(
            ss.str(), 1, boost::bind(&RosNode::_RosTwistCallback, this, _1, i),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
    }
}
void RosNode::_setArena2dResPub()
{
    for (int i = 0; i < m_num_envs; i++)
    {
        std::stringstream ss;
        ss << "env_" << i << "/response";
        m_arena2d_pubs.push_back(
            m_nh_ptr->advertise<arena2d_msgs::Arena2dResp>(ss.str(), 1));
    }
}

void RosNode::publishStates(const bool *dones, float mean_reward,
                            float mean_sucess)
{
    for (int idx_env = 0; idx_env < m_num_envs; idx_env++)
    {
        // if any env need reset and not current one just skip.
        if (m_any_env_reset && !m_envs_reset[idx_env])
        {
            continue;
        }
        arena2d_msgs::Arena2dResp resp;
        // set laser scan data
        int num_sample;
        auto laser_data = m_envs[idx_env].getScan(num_sample);
        for (size_t i = 0; i < num_sample; i++)
        {
            resp.observation.ranges.push_back(laser_data[i]);
        }

        // set goal position & distance and angle from goal
        float goal_x = 0, goal_y = 0;
        float angle = 0, distance = 0;
        m_envs[idx_env].getGoalDistance(distance, angle, goal_x, goal_y);
        resp.goal_pos[0] = static_cast<double>(goal_x);
        resp.goal_pos[1] = static_cast<double>(goal_y);
        resp.goal_info[0] = static_cast<double>(distance);

        float angle_rad = angle * (PI/180);
        float angle_final = 0.0;
        if(angle_rad < 0){
            angle_final = angle_rad + PI;
        }
        else{
            angle_final = angle_rad - PI;
        }

        resp.goal_info[1] = static_cast<double>(angle_final);
        // robot theta is in radian
        float robot_x, robot_y, robot_theta;
        m_envs[idx_env].getRobotPos(robot_x, robot_y, robot_theta);
        resp.robot_pos.x = static_cast<double>(robot_x);
        resp.robot_pos.y = static_cast<double>(robot_y);
        resp.robot_pos.theta = static_cast<double>(robot_theta);
        // obstacle position

        // cout << "angle is: " << angle << endl;
        // cout << "angle_rad is: " << angle_rad << endl << endl;
        // cout << "angle_final is: " << angle_final << endl << endl;
        // cout << "robot_theta is: " << robot_theta << endl;
        // float y_relative = goal_y - robot_y;
        // float x_relative = goal_x - robot_x;
        // float rho =  pow((pow(x_relative,2)+pow(y_relative,2)),0.5);
        // cout << "distance is: " << distance << endl;
        // cout << "rho is: " << rho << endl;

        // double theta = fmod((atan2(y_relative,x_relative) - robot_theta + 4 * PI),(2 * PI)) - PI;
        // cout << "theta is: " << theta << endl << endl;
        string _level = _SETTINGS->stage.initial_level;
        string::size_type idx_robot, idx_human;
        idx_robot = _level.find("dynamic");
        idx_human = _level.find("human");
        if (idx_robot != string::npos)
        {
            std::vector<float> robot_obstacle_data;
            m_envs[idx_env].getRobotObstaclePosition(robot_obstacle_data);
            for (auto i = 0; i < robot_obstacle_data.size(); i++)
            {
                resp.robot_obstacle_pos[i] =
                    static_cast<double>(robot_obstacle_data[i]);
            }
        }
        if (idx_human != string::npos)
        {
            std::vector<float> human_obstacle_data;
            m_envs[idx_env].getHumanObstaclePosition(human_obstacle_data);
            for (auto i = 0; i < human_obstacle_data.size(); i++)
            {
                resp.human_obstacle_pos[i] =
                    static_cast<double>(human_obstacle_data[i]);
            }
        }
        // add last_action als response
        resp.last_action.linear = last_linear;
        resp.last_action.angular = last_angular;
        // TODO Whats is additional data? add it and change the msg type if needed

        resp.reward = m_envs[idx_env].getReward();
        resp.done = dones[idx_env];
        resp.mean_reward = mean_reward;
        resp.mean_success = mean_sucess;

        m_arena2d_pubs[idx_env].publish(resp);
    }
    if (!m_any_env_reset)
    {
        ROS_DEBUG("published states");
    }
    else
    {
        m_any_env_reset = false;
        for (int i = 0; i < m_envs_reset.size(); i++)
        {
            m_envs_reset[i] = false;
        }
        ROS_DEBUG("env already reset");
    }
    m_num_ros_agent_req_msgs_received = 0;
}

void RosNode::_RosAgentReqCallback(
    const arena2d_msgs::RosAgentReq::ConstPtr &req_msg, int idx_env)
{
    if (req_msg->arena2d_sim_close)
    {
        if (m_env_close == 0)
        {
            m_num_ros_agent_req_msgs_received = 0; // reset the counter
        }
        m_env_close += 1;
    }
    if (req_msg->env_reset == 1 )
    {
        m_envs_reset[idx_env] = true;
        m_any_env_reset = true;
        m_envs_reset_list.push_back(idx_env);
        ROS_DEBUG_STREAM("env " << idx_env << " request reset");
    }
    else
    {
        m_actions_buffer[idx_env]->linear = req_msg->action.linear;
        m_actions_buffer[idx_env]->angular = req_msg->action.angular;
        ROS_DEBUG_STREAM("env " << idx_env << " message received");
    }
    m_num_ros_agent_req_msgs_received++;

    last_linear = req_msg->action.linear;
    last_angular = req_msg->action.angular;
}

void RosNode::_RosTwistCallback(const geometry_msgs::Twist::ConstPtr &req_msg,
                                int idx_env)
{
    m_actions_buffer[idx_env]->linear = req_msg->linear.x;
    m_actions_buffer[idx_env]->angular = req_msg->angular.z;
    m_num_ros_agent_req_msgs_received++;
    last_linear = req_msg->linear.x;
    last_angular = req_msg->angular.z;
}

RosNode::Status RosNode::getActions(Twist *robot_Twist, bool *ros_envs_reset,
                                    float waitTime = 0)
{
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(waitTime));
    if (m_any_env_reset)
    {
        for (int i = 0; i < m_num_envs; i++)
        {
            ros_envs_reset[i] = m_envs_reset[i];
        }
        return RosNode::Status::ENV_RESET;
    }
    if (m_num_ros_agent_req_msgs_received == m_num_envs)
    {
        if (m_env_close == 0) // all env are normal
        {
            for (int i = 0; i < m_num_envs; i++)
            {
                robot_Twist[i].angular = m_actions_buffer[i]->angular;
                robot_Twist[i].linear = m_actions_buffer[i]->linear;
            }
            ROS_DEBUG("Requests from all env received!");
            return RosNode::Status::ALL_AGENT_MSG_RECEIVED;
        }
        else if (m_env_close == m_num_envs)
        {
            return RosNode::Status::SIM_CLOSE; // only all env wrapper requiring
                                               // close is considered legal
        }
        else
        {
            return RosNode::Status::BAD_MESSAGE;
        }
    }
    else
    {
        return RosNode::Status::NOT_ALL_AGENT_MSG_RECEIVED;
    }
}

void RosNode::waitConnection()
{
    int n_connected = 0;
    for (auto &sub : m_ros_agent_subs)
    {
        if (sub.getNumPublishers() != 0)
        {
            n_connected++;
        }
    }
    for (auto &pub : m_arena2d_pubs)
    {
        if (pub.getNumSubscribers() != 0)
        {
            n_connected++;
        }
    }
    if (n_connected != m_arena2d_pubs.size() + m_ros_agent_subs.size())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    else
    {
        m_env_connected = true;
        ROS_INFO("All agents successfully connected!");
    }
}

bool RosNode::DeleteModelCallback(
    arena2d_msgs::DeleteModel::Request &request,
    arena2d_msgs::DeleteModel::Response &response)
{
    ROS_DEBUG_NAMED("Service", "Model delete requested with name(\"%s\")",
                    request.name.c_str());
    // current only service for delete "obstacle"
    try
    {
        if (request.name == "all")
        {
            if (!m_envs_reset_list.empty())
            {
                _SETTINGS->stage.num_dynamic_obstacles = 0;
                _SETTINGS->stage.num_obstacles = 0;
                int env = m_envs_reset_list.front();
                m_envs[env].reset(false);
                // cout << "env: " << env << " be deleted" << endl;
            }
            response.success = true;
            response.message = "";
        }
        else
        {
            response.success = false;
            response.message =
                "Wrong model type for 'delete model', please try again";
        }
    }
    catch (const std::exception &e)
    {
        response.success = false;
        response.message = std::string(e.what());
    }

    return true;
}

bool RosNode::MoveModelCallback(arena2d_msgs::MoveModel::Request &request,
                                arena2d_msgs::MoveModel::Response &response)
{
    ROS_DEBUG_NAMED("Service", "Model move requested with name(\"%s\")",
                    request.name.c_str());

    // currently only service for move "robot"
    try
    {
        if (request.name == "robot")
        {
            b2Vec2 pose;
            pose.x = request.pose.x;
            pose.y = request.pose.y;
            float theta = request.pose.theta;

            if (!m_envs_reset_list.empty())
            {
                int env = m_envs_reset_list.front();
                m_envs[env].getRobot()->reset(pose, theta);
                // cout << "env: " << env << " be moved" << endl;
            }

            response.success = true;
            response.message = "";
        }
        else
        {
            response.success = false;
            response.message = "Wrong model type for 'move model', please try again";
        }
    }
    catch (const std::exception &e)
    {
        response.success = false;
        response.message = std::string(e.what());
    }

    return true;
}

bool RosNode::SpawnModelCallback(arena2d_msgs::SpawnModel::Request &request,
                                 arena2d_msgs::SpawnModel::Response &response)
{
    ROS_DEBUG_NAMED("Service", "Model spawn requested with name(\"%s\")",
                    request.name.c_str());

    try
    {
        // if request name have prefix static, generate a random static obstacle
        if (boost::starts_with(request.name, "static"))
        {
            if (!m_envs_reset_list.empty())
            {
                _SETTINGS->stage.num_obstacles++;
                _SETTINGS->stage.min_obstacle_size = request.min_radius*2;
                _SETTINGS->stage.max_obstacle_size = request.max_radius*2;
                int env = m_envs_reset_list.front();
                m_envs[env].reset(false);
                _SETTINGS->stage.num_obstacles--;
                // cout << "env: " << env << " spawn a static obscale" << endl;
            }
            _SETTINGS->stage.num_obstacles++;

            response.success = true;
            response.message = "";
        }
        // if request name have prefix dynamic, generate a random dynamic obstacle
        else if (boost::starts_with(request.name, "dynamic"))
        {
            if (!m_envs_reset_list.empty())
            {
                _SETTINGS->stage.num_dynamic_obstacles++;
                _SETTINGS->stage.min_dynamic_obstacle_size = request.min_radius*2;
                _SETTINGS->stage.dynamic_obstacle_size = request.max_radius*2;
                _SETTINGS->stage.obstacle_speed = request.linear_vel;
                _SETTINGS->stage.obstacle_angular_max = request.angular_vel_max / (PI/180);
                int env = m_envs_reset_list.front();
                m_envs[env].reset(false);
                _SETTINGS->stage.num_dynamic_obstacles--;
                // cout << "env: " << env << " spawn a dynamic obscale" << endl;
            }
            _SETTINGS->stage.num_dynamic_obstacles++;
            response.success = true;
            response.message = "";
        }
        else
        {
            response.success = false;
            response.message = "Wrong model type for 'spawn model', please try again";
        }
        // current only service for delete obstacle
    }
    catch (const std::exception &e)
    {
        response.success = false;
        response.message = std::string(e.what());
    }

    return true;
}

// generate pedestrian in waypoint mode -- Deserted for training
bool RosNode::SpawnPedestrianCallback(
    arena2d_msgs::SpawnPeds::Request &request,
    arena2d_msgs::SpawnPeds::Response &response)
{
    try
    {
        std::vector<b2Vec2> waypoints;
        b2Vec2 p;
        for (std::vector<geometry_msgs::Point>::iterator it =
                 request.waypoints.begin();
             it != request.waypoints.end(); ++it)
        {
            cout << "waypoint is " << it->x << ", " << it->y << endl;
            p.x = it->x;
            p.y = it->y;
            waypoints.push_back(p);
        }
        for (int idx_env = 0; idx_env < m_num_envs; idx_env++)
        {
            m_envs[idx_env].getLevel()->waypointsStore(waypoints);
            _SETTINGS->stage.num_dynamic_obstacles++;
            m_envs[idx_env].reset(false);
            _SETTINGS->stage.num_dynamic_obstacles--;
        }
        _SETTINGS->stage.num_dynamic_obstacles++;
        response.success = true;
        response.message = "";
    }
    catch (const std::exception &e)
    {
        response.success = false;
        response.message = std::string(e.what());
    }

    return true;
}

bool RosNode::PauseCallback(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response)
{
    pause_flag = true;
    return true;
}

bool RosNode::UnpauseCallback(std_srvs::Empty::Request &request,
                              std_srvs::Empty::Response &response)
{
    m_envs_reset_list.pop_front();
    pause_flag = false;
    return true;
}
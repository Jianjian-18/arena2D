#include "LevelScenario.hpp"

std::shared_ptr<nav_msgs::OccupancyGrid> Scenario::m_scenario = nullptr;
std::unique_ptr<ros::NodeHandle> Scenario::m_nh = nullptr;

LevelScenario::LevelScenario(const LevelDef &d, bool dynamic, bool human):
Level(d), _dynamic(dynamic), _human(human), wanderers(d)
{   
    _human=human;
    _dynamic=dynamic;
    _init_reset=true;
    _n_non_clear_bodies = 0;
    _occupancygrid_ptr = Scenario::getMap(_SETTINGS->stage.scenario_ros_service_name);
    ROS_INFO("load map start");
    loadScenario();
    ROS_INFO("loaded map!");
    
}

void LevelScenario::reset(bool robot_position_reset)
{
    ROS_DEBUG("reset start!");
    lazyclear();
    if(_human )
        wanderers.freeWanderers();
    if(_dynamic)
        wanderers.freeRobotWanderers();

    // get constants
	const float half_goal_size = _SETTINGS->stage.goal_size/2.f;
	const float dynamic_radius = _SETTINGS->stage.dynamic_obstacle_size/2.f;
	const float dynamic_speed = _SETTINGS->stage.obstacle_speed;
	const int num_obstacles = _SETTINGS->stage.num_obstacles;
	const int num_dynamic_obstacles = _SETTINGS->stage.num_dynamic_obstacles;
	const float min_obstacle_radius = _SETTINGS->stage.min_obstacle_size/2;
	const float max_obstacle_radius = _SETTINGS->stage.max_obstacle_size/2;
    const auto &info = _occupancygrid_ptr->info;
    const float half_height = info.resolution * info.height / 2;
    const float half_width = info.resolution * info.width / 2;
	const zRect main_rect(0,0, half_width, half_height);
	const zRect big_main_rect(0, 0, half_width+max_obstacle_radius, half_height+max_obstacle_radius);
    double fixed_x = 0.0, fixed_y = 0.0;
    ros::param::get("scenerio/goal_x", fixed_x);
    ros::param::get("scenerio/goal_y", fixed_y);
    const zRect goal_rect(fixed_x, fixed_y, 0, 0);
    b2Vec2 pos;
    ros::param::get("scenerio/robot_x", pos.x);
    ros::param::get("scenerio/robot_y", pos.y);
    


	


	// create static obstacles

   


    if (_init_reset)
    {
        bool scenerio = false;
        ros::param::get("scenerio/scenerio", scenerio);
        if(!scenerio){
            _goalSpawnArea.addQuadTree(main_rect, _levelDef.world, COLLIDE_CATEGORY_STAGE,
                                    LEVEL_RANDOM_GOAL_SPAWN_AREA_BLOCK_SIZE, half_goal_size);
        }
        else{
            _goalSpawnArea.addQuadTree(goal_rect, _levelDef.world, COLLIDE_CATEGORY_STAGE,
                                    LEVEL_RANDOM_GOAL_SPAWN_AREA_BLOCK_SIZE, half_goal_size);            
        }       
        // calculating goal spawn area
        _goalSpawnArea.calculateArea();
        ROS_INFO("calculating the respawn area for static obstacles...");
        _staticSpawn.addCheeseRect(main_rect, _levelDef.world, COLLIDE_CATEGORY_PLAYER, max_obstacle_radius);
        _staticSpawn.calculateArea();        
        ROS_INFO("calculation the respawn area for static obstacles is done");
    }

    // if (robot_position_reset)
    // {
    //     resetRobotToCenter();
    //     // _levelDef.robot->reset(pos, 0);
    //     // _levelDef.robot->reset(pos, f_frandomRange(0, 2 * M_PI));
    // }
    // else{
    //     robotSpawnUntilValid();
    // }

    //random reset robot
    robotSpawnUntilValid();
        
    // dynamic obstacles
	for(int i = 0; i < num_obstacles; i ++){
        staticObstacleSpawnUntilValid();
	}
    ROS_DEBUG("static obstacles created!");   
    if (_dynamic || _human)
    {
        if (_init_reset)
        {
            ROS_INFO("calculating the respawn area for dynamic obstacles, it may take a while and the GUI is in black "
                     "screen...");
            _dynamicSpawn.clear();
            _dynamicSpawn.addCheeseRect(main_rect, _levelDef.world, COLLIDE_CATEGORY_STAGE | COLLIDE_CATEGORY_PLAYER,
                                        dynamic_radius);
            _dynamicSpawn.calculateArea();
            ROS_INFO("calculation the respawn area for dynamic obstacles is done");
            _init_reset = false;
        }
        for(int i = 0; i < num_dynamic_obstacles; i ++){
            dynamicObstacleSpawnUntilValid();
        }
        wanderers.resetInfo();       
        ROS_DEBUG("dynamic obstacles created!");             
        // wanderers.reset(_dynamicSpawn, _dynamic, _human);
    }
    randomGoalSpawnUntilValid();
    ROS_DEBUG("goal spawned");
}


void LevelScenario::renderGoalSpawn()
{
    Level::renderGoalSpawn();
    Z_SHADER->setColor(zColor(0.1, 0.9, 0.0, 0.5));
    _dynamicSpawn.render();
}

void LevelScenario::loadScenario()
{   
    
    b2Assert(_occupancygrid_ptr);
    const auto &info = _occupancygrid_ptr->info;
    const auto &data = _occupancygrid_ptr->data;
    uint32 cols = info.width;
    uint32 rows = info.height;
    float resolution = info.resolution;
    // get the position of the left-upper cell, we assume the origin of the coordinate system is the center of the map
    b2Vec2 lower_left_pos(-((cols >> 1) - ((cols & 1) ^ 1) / 2.f) * resolution,
                          -((rows >> 1) - ((rows & 1) ^ 1) / 2.f) * resolution);

    // get map with line segments
    b2BodyDef b;
    b.type = b2_staticBody;
    b2Body *body = _levelDef.world->CreateBody(&b);
    
    // create method of get line segment
    auto add_edge = [&](double x1, double y1, double x2, double y2) {
            b2EdgeShape edge;

            edge.Set(b2Vec2(lower_left_pos.x + resolution * x1, lower_left_pos.y + resolution * y1),
                     b2Vec2(lower_left_pos.x + resolution * x2, lower_left_pos.y + resolution * y2));

            b2FixtureDef fixture_def;
            fixture_def.shape = &edge;
            fixture_def.filter.categoryBits = COLLIDE_CATEGORY_STAGE;
            fixture_def.friction = LEVEL_STATIC_FRICTION;
            fixture_def.restitution = LEVEL_STATIC_RESTITUTION;
            body->CreateFixture(&fixture_def);
    };

    // scenario in CV matrix
    cv::Mat scenario(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            scenario.at<uint8>(i, j) = (data[i * cols + j] == 100 || data[i * cols + j] == -1) ? 255 : 0;
        }
    }

    _occupancy_map=scenario.clone();

    /*
    // Create FLD detector
    // Param               Default value   Description
    // length_threshold    10            - Segments shorter than this will be discarded
    // distance_threshold  1.41421356    - A point placed from a hypothesis line
    //                                     segment farther than this will be
    //                                     regarded as an outlier
    // canny_th1           50            - First threshold for
    //                                     hysteresis procedure in Canny()
    // canny_th2           50            - Second threshold for
    //                                     hysteresis procedure in Canny()
    // canny_aperture_size 3             - Aperturesize for the sobel
    //                                     operator in Canny()
    // do_merge            false         - If true, incremental merging of segments
    //                                     will be perfomred
    // int length_threshold = 1;
    // float distance_threshold = 1.41421356f;
    // double canny_th1 = 0.1;
    // double canny_th2 = 0.2;
    // int canny_aperture_size = 3;
    // bool do_merge = false;
    // auto fld = cv::ximgproc::createFastLineDetector(length_threshold,
    //                                                 distance_threshold, canny_th1, canny_th2, canny_aperture_size,
    //                                                 do_merge);
    // vector<cv::Vec4f> lines_fld;
    // fld->detect(scenario, lines_fld);
    // for (const auto &line : lines_fld)
    // {
    //     add_edge(line[0], line[1], line[2], line[3]);
    // }
    */
    
    // loop through all the rows, looking at 2 at once
    for (int i = 0; i < scenario.rows - 1; i++)
    {
        cv::Mat row1 = scenario.row(i);
        cv::Mat row2 = scenario.row(i + 1);
        cv::Mat diff;

        // if the two row are the same value, there is no edge
        // if the two rows are not the same value, there is an edge
        // result is still binary, either 255 or 0
        cv::absdiff(row1, row2, diff);

        int start = 0;
        bool started = false;

        // find all the walls, put the connected walls as a single line segment
        for (unsigned int j = 0; j <= diff.total(); j++)
        {
            bool edge_exists = false;
            if (j < diff.total())
            {
                edge_exists = diff.at<uint8_t>(0, j); // 255 maps to true
            }

            if (edge_exists && !started)
            {
                start = j;
                started = true;
            }
            else if (started && !edge_exists)
            {
                add_edge(start, i, j, i);
                // add_edge(start, i + 1, j, i + 1);
                started = false;
            }
        }
    }
    // loop through all the columns, looking at 2 at once
    for (int i = 0; i < scenario.cols - 1; i++)
    {
        cv::Mat col1 = scenario.col(i);
        cv::Mat col2 = scenario.col(i + 1);
        cv::Mat diff;

        cv::absdiff(col1, col2, diff);

        int start = 0;
        bool started = false;

        for (unsigned int j = 0; j <= diff.total(); j++)
        {
            bool edge_exists = false;
            if (j < diff.total())
            {
                edge_exists = diff.at<uint8_t>(j, 0);
            }

            if (edge_exists && !started)
            {
                start = j;
                started = true;
            }
            else if (started && !edge_exists)
            {
                add_edge(i, start, i, j);
                // add_edge(i + 1, start, i + 1, j);

                started = false;
            }
        }
    }
    _n_non_clear_bodies++;
    _bodyList.push_back(body);

}

void LevelScenario::lazyclear()
{
    // only free the bodies that not belong to static map.
    int i = 0, size = _bodyList.size();
    for (auto it = _bodyList.begin(); it != _bodyList.end();)
    {
        if (i++ >= _n_non_clear_bodies)
        {
            _levelDef.world->DestroyBody(*it);
            it = _bodyList.erase(it);
        }
        else
        {
            it++;
        }
    }
}

float LevelScenario::getReward()
{
	float reward = 0;
	_closestDistance_old.clear();
	_closestDistance.clear();

	//reward for observed humans inside camera view of robot (number limited by num_obs_humans)
	if(_SETTINGS->training.reward_function == 1 || _SETTINGS->training.reward_function == 4){
		wanderers.get_old_observed_distances(_closestDistance_old);
		wanderers.get_observed_distances(_closestDistance);
	}
	//reward for all humans in the level
	else if(_SETTINGS->training.reward_function == 2 || _SETTINGS->training.reward_function == 3){
		wanderers.get_old_distances(_closestDistance_old);
		wanderers.get_distances(_closestDistance);
	}
	

	for(int i = 0; i < _closestDistance_old.size(); i++){
		float distance_after = _closestDistance[i];
		float distance_before = _closestDistance_old[i];
		// give reward only if current distance is smaller than the safety distance
		if(distance_after < _SETTINGS->training.safety_distance_human){
 			//give reward for distance to human decreased/increased linearly depending on the distance change 
			if(_SETTINGS->training.reward_function == 3 || _SETTINGS->training.reward_function == 4){
				if(distance_after < distance_before){
					reward += _SETTINGS->training.reward_distance_to_human_decreased * (distance_before - distance_after);
				}else if(distance_after > distance_before){
					reward += _SETTINGS->training.reward_distance_to_human_increased * (distance_after - distance_before);
				}
			}
			//give constant reward for distance to human decreased/increased
			else{
                reward += _SETTINGS->training.safety_distance_human;
				if(distance_after < distance_before){
					reward += _SETTINGS->training.reward_distance_to_human_decreased;
				}else if(distance_after > distance_before){
					reward += _SETTINGS->training.reward_distance_to_human_increased;
				}
			}
		}
	}
	return reward;
}
void LevelScenario::robotSpawnUntilValid(RectSpawn * goal_spawn)
{   
    const auto &info = _occupancygrid_ptr->info;
    const auto &data = _occupancygrid_ptr->data;
    uint32 cols = info.width;
    uint32 rows = info.height;
    float resolution = info.resolution;
    b2Vec2 lower_left_pos(-((cols >> 1) - ((cols & 1) ^ 1) / 2.f) * resolution,
                          -((rows >> 1) - ((rows & 1) ^ 1) / 2.f) * resolution);

    RectSpawn * spawn = &_goalSpawnArea;
	if(goal_spawn != NULL)// use custom goal spawn
	{
		spawn = goal_spawn;
	}
	// spawn goal at random position
	b2Vec2 spawn_position(0,0);
	int count = 0;
    bool occupied;
    bool point_out;
    b2Vec2 coord;
    int i,j;
	do{
        point_out = false;
        occupied=false;
		spawn->getRandomPoint(spawn_position);
        
        coord=(spawn_position-lower_left_pos);
        
        i=floor(coord.y/resolution);
        j=floor(coord.x/resolution);
        if(i < 0 || i > rows || j < 0 || j > cols) point_out = true;
        int point = _occupancy_map.at<uint8>(i, j);
        if ( point == 255 || point_out){
            occupied=true;
        }
        
		count++;
        
	}while((occupied) && count < 100);
	_levelDef.robot->reset(spawn_position, f_frandomRange(0, 2 * M_PI));
   
}

void LevelScenario::randomGoalSpawnUntilValid(RectSpawn * goal_spawn)
{   
    const auto &info = _occupancygrid_ptr->info;
    const auto &data = _occupancygrid_ptr->data;
    uint32 cols = info.width;
    uint32 rows = info.height;
    float resolution = info.resolution;
    b2Vec2 lower_left_pos(-((cols >> 1) - ((cols & 1) ^ 1) / 2.f) * resolution,
                          -((rows >> 1) - ((rows & 1) ^ 1) / 2.f) * resolution);

    
    RectSpawn * spawn = &_goalSpawnArea;
	if(goal_spawn != NULL)// use custom goal spawn
	{
		spawn = goal_spawn;
	}

	b2Vec2 robot_position = _levelDef.robot->getPosition();
	// spawn goal at random position
	b2Vec2 spawn_position(0,0);
	int count = 0;
    bool occupied;
    bool point_out;
    b2Vec2 coord;
    int i,j;
	do{
        point_out = false;
        occupied=false;
		spawn->getRandomPoint(spawn_position);
        
        coord=(spawn_position-lower_left_pos);
        
        i=floor(coord.y/resolution);
        j=floor(coord.x/resolution);
        
        if(i < 0 || i > rows || j < 0 || j > cols) point_out = true;
        int point = _occupancy_map.at<uint8>(i, j);
        if (point == 255 || point_out){
            occupied=true;
        }
        
		count++;
        
	}while((!checkValidGoalSpawn(robot_position, spawn_position) || occupied) && count < 100);
    // cout << "find goal in freespace  within count"<<count<<"   Position "<<"x="<<spawn_position.x<<" y="<<spawn_position.y <<endl;
    // ROS_DEBUG_STREAM("find goal in freespace  within count"<<count<<"   Position "<<"x="<<spawn_position.x<<" y="<<spawn_position.y);
	spawnGoal(spawn_position);
   
}

void LevelScenario::dynamicObstacleSpawnUntilValid(){
    const auto &info = _occupancygrid_ptr->info;
    const auto &data = _occupancygrid_ptr->data;
    uint32 cols = info.width;
    uint32 rows = info.height;
    float resolution = info.resolution;
    b2Vec2 lower_left_pos(-((cols >> 1) - ((cols & 1) ^ 1) / 2.f) * resolution,
                          -((rows >> 1) - ((rows & 1) ^ 1) / 2.f) * resolution);
	int count = 0;
    bool occupied;
    b2Vec2 robot_position = _levelDef.robot->getPosition();   
    b2Vec2 coord;
    b2Vec2 p;
    int i,j;
	do{
        occupied=false;
		_dynamicSpawn.getRandomPoint(p);
        
        coord=(p-lower_left_pos);
        
        i=floor(coord.y/resolution);
        j=floor(coord.x/resolution);
        
        int point = _occupancy_map.at<uint8>(i, j);
        if ( point == 255){
            occupied=true;
        }
        
		count++;
        
	}while((!checkValidGoalSpawn(robot_position, p) || occupied) && count < 1000);
    wanderers.resetSingleRandomObstacle(p, _dynamicSpawn, _dynamic, _human);

    // If the number of obstacles increases, the counter upper limit also needs to be increased
    // ROS_DEBUG_STREAM("find all dynamic obstacles in freespace  within count "<< count);
    // std::cout<<"find all dynamic obstacles in freespace  within count "<< count << std::endl;
    // cout << "find dynamic obstacle in freespace  within count"<<count<<"  Position "<<"x="<<p.x<<" y="<<p.y <<endl << endl;
}

void LevelScenario::staticObstacleSpawnUntilValid(){
    const auto &info = _occupancygrid_ptr->info;
    const auto &data = _occupancygrid_ptr->data;
    uint32 cols = info.width;
    uint32 rows = info.height;
    float resolution = info.resolution;
    b2Vec2 lower_left_pos(-((cols >> 1) - ((cols & 1) ^ 1) / 2.f) * resolution,
                          -((rows >> 1) - ((rows & 1) ^ 1) / 2.f) * resolution);
	int count = 0;
    bool occupied;
    b2Vec2 robot_position = _levelDef.robot->getPosition();   
    b2Vec2 coord;
    b2Vec2 p;
    int i,j;
	do{
        occupied=false;
		_staticSpawn.getRandomPoint(p);
        
        coord=(p-lower_left_pos);
        
        i=floor(coord.y/resolution);
        j=floor(coord.x/resolution);
        
        int point = _occupancy_map.at<uint8>(i, j);
        if ( point == 255){
            occupied=true;
        }
        
		count++;
        
	}while((!checkValidGoalSpawn(robot_position, p) || occupied) && count < 1000);
    // cout << "find static obstacle in freespace  within count"<<count<<"  Position "<<"x="<<p.x<<" y="<<p.y << endl<< endl;
    // ROS_DEBUG_STREAM("find static obstacle in freespace  within count"<<count<<"   Position "<<"x="<<p.x<<" y="<<p.y);
    zRect aabb;
    addRandomShape(p, _SETTINGS->stage.min_obstacle_size/2, _SETTINGS->stage.max_obstacle_size/2, &aabb);	
}

void LevelScenario::waypointsStore(std::vector<b2Vec2> waypoints){
    wanderers.getWaypointsList().push_back(waypoints);
}

void LevelScenario::waypointsClear(){
    wanderers.getWaypointsList().clear();
}
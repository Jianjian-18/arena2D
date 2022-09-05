/* Author: Cornelius Marx */
#include "Arena.hpp"
void Arena::update()
{
	// update key control
	int action = -1;
	if (!_trainingMode)
	{
		if (_playSimulation)
		{
			action = Robot::STOP;
		}
		if (_keysPressed[UP])
		{
			action = Robot::FORWARD;
			if (_keysPressed[LEFT])
			{
				action = Robot::FORWARD_LEFT;
			}
			else if (_keysPressed[RIGHT])
			{
				action = Robot::FORWARD_RIGHT;
			}
		}
		else if (_keysPressed[DOWN])
		{
			action = Robot::BACKWARD;
		}
		else
		{
			if (_keysPressed[LEFT])
			{
				action = Robot::FORWARD_STRONG_LEFT;
			}
			else if (_keysPressed[RIGHT])
			{
				action = Robot::FORWARD_STRONG_RIGHT;
			}
		}
		if (action != -1)
		{
			for (int i = 0; i < _numEnvs; i++)
			{
				Robot::getActionTwist((Robot::Action)action, _actions[i]);
			}
		}
	}
	int episodes_before = _episodeCount;
	if (_trainingMode && _pyAgentUsed)
	{
		_agentMeasure.startTime();
		if (_agentFuncs[PYAGENT_FUNC_PRE_STEP] != NULL)
		{
			// creating list
			PyObject *args = PyTuple_New(1);
			PyTuple_SetItem(args, 0, packAllPyObservation());
			PyObject *result = PyObject_CallObject(_agentFuncs[PYAGENT_FUNC_PRE_STEP], args);
			Py_DECREF(args);
			if (result == NULL)
			{
				PyErr_Print();
				ERROR_F("Call to function '%s' in Python agent failed", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP]);
				cmdStopTraining(ConsoleParameters(0, NULL));
			}
			else
			{
				bool return_error = false;
				if (PyList_Check(result))
				{ // get multiple actions
					if (PyList_Size(result) != _numEnvs)
					{ // check number of elements
						ERROR_F("Call to function '%s' in Python agent failed: Size of returned list (%d) does not match number of environments (%d)!",
								PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP], (int)PyList_Size(result), _numEnvs);
						cmdStopTraining(ConsoleParameters(0, NULL));
					}
					else
					{
						action = 0;
						for (int i = 0; i < _numEnvs; i++)
						{
							PyObject *item = PyList_GetItem(result, i);
							if (PyTuple_Check(item))
							{
								_actions[i].linear = PyFloat_AsDouble(PyTuple_GetItem(item, 0));
								_actions[i].angular = PyFloat_AsDouble(PyTuple_GetItem(item, 1));
							}
							else if (PyLong_Check(item))
							{
								Robot::getActionTwist((Robot::Action)PyLong_AsLong(item), _actions[i]);
							}
							else
							{
								return_error = true;
								break;
							}
						}
					}
				}
				else
				{
					Twist t;
					if (PyLong_Check(result))
					{ // get single action
						action = 0;
						Robot::getActionTwist((Robot::Action)PyLong_AsLong(result), t);
					}
					else if (PyTuple_Check(result))
					{ // get single twist (tuple)
						action = 0;
						t.linear = PyFloat_AsDouble(PyTuple_GetItem(result, 0));
						t.angular = PyFloat_AsDouble(PyTuple_GetItem(result, 1));
					}
					else
					{
						return_error = true;
					}

					if (action >= 0)
					{
						for (int i = 0; i < _numEnvs; i++)
						{
							_actions[i] = t;
						}
					}
				}
				Py_DECREF(result);
				if (action < 0)
				{
					ERROR_F("Call to function '%s' in Python agent failed: Expected List or Long for return value!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP]);
					cmdStopTraining(ConsoleParameters(0, NULL));
				}

				if (return_error)
				{
					ERROR_F("Unexpected return type from function '%s' in Python agent: Expected Long or Tuple!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP]);
					cmdStopTraining(ConsoleParameters(0, NULL));
				}
			}
		}
		_agentMeasure.endTime();
	}

	_physicsTimer.checkLastUpdate();

	if (action >= 0)
	{
		// check if episodes are already done
		for (int i = 0; i < _numEnvs; i++)
		{
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if (s != Environment::RUNNING)
			{
				_dones[i] = true;
			}
			else
			{
				_dones[i] = false;
			}
		}
		// threaded simulation step
		_simulationMeasure.startTime();
		for (int i = 0; i < _numThreads; i++)
		{
			_threads[i].step();
		}

		for (int i = 0; i < _numThreads; i++)
		{
			_threads[i].wait_finish();
		}
		_simulationMeasure.endTime();

		// check whether episodes has just ended
		bool episode_over = false;
		for (int i = 0; i < _numEnvs; i++)
		{
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if (s != Environment::RUNNING && !_dones[i])
			{ // episode is now over and has not been over before
				_dones[i] = true;
				// adding success value to success buffer
				_meanSuccess.push((s == Environment::POSITIVE_END) ? 1 : 0);
				_meanSuccess.calculateMean();
				
				// adding reward to total reward buffer
				_meanReward.push(_envs[i].getTotalReward());
				_meanReward.calculateMean();

				_episodeCount++;

				// show results
				printEpisodeResults(_envs[i].getTotalReward());

				episode_over = true;
			}
		}
		if (episode_over)
		{
			if (_SETTINGS->video.enabled)
			{
				refreshEpisodeCounter();
				refreshRewardCounter();
			}
		}

		// measuring FPS
		_physicsTimer.update(false);

		// call agents post step function
		bool reset_envs = true;
		if (_trainingMode && _pyAgentUsed)
		{
			_agentPostMeasure.startTime();
			if (_agentFuncs[PYAGENT_FUNC_POST_STEP] != NULL)
			{
				PyObject *args = PyTuple_New(5);
				PyTuple_SetItem(args, 0, packAllPyObservation());					  //observation
				PyTuple_SetItem(args, 1, packAllPyRewards());						  //reward
				PyTuple_SetItem(args, 2, packAllPyDones());							  //done
				PyTuple_SetItem(args, 3, PyFloat_FromDouble(_meanReward.getMean()));  //mean reward
				PyTuple_SetItem(args, 4, PyFloat_FromDouble(_meanSuccess.getMean())); //mean success
				PyObject *result = PyObject_CallObject(_agentFuncs[PYAGENT_FUNC_POST_STEP], args);
				Py_DECREF(args);
				if (result == NULL)
				{
					PyErr_Print();
					ERROR_F("Call to function '%s' in Python agent failed", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_POST_STEP]);
					cmdStopTraining(ConsoleParameters(0, NULL));
				}
				else
				{
					int done = PyLong_AsLong(result);
					Py_DECREF(result);
					if (done > 0)
					{ // stop training
						cmdStopTraining(ConsoleParameters(0, NULL));
						INFO("Training done!");
					}
					else if (done < 0)
					{ // do not reset environments
						reset_envs = false;
					}
				}
			}
			_agentPostMeasure.endTime();
		}

		// reset environments if allowed by agent
		if (reset_envs)
		{
			for (int i = 0; i < _numEnvs; i++)
			{
				Environment::EpisodeState s = _envs[i].getEpisodeState();
				if (s != Environment::RUNNING)
				{
					_levelResetMeasure.startTime();
					_envs[i].reset(false);
					_levelResetMeasure.endTime();
				}
			}
			if (_SETTINGS->video.enabled && !_videoDisabled)
			{
				refreshLevelResetTime();
			}
		}

		// write to csv file if episode count changed
		if (!_noTrainingRecord && _trainingMode && _pyAgentUsed && _episodeCount != episodes_before)
		{
			// write header first?
			const bool write_header = (_csvWriter.getNumLines() == 0);
			std::vector<const char *> names(3);
			// default metrics
			if (write_header)
			{
				names[0] = "Episodes";
				names[1] = "Success";
				names[2] = "Mean Reward";
			}
			std::vector<float> values(3);
			values[0] = (float)_episodeCount;
			values[1] = _meanSuccess.getMean();
			values[2] = _meanReward.getMean();
			// call get_stats python function
			if (_agentFuncs[PYAGENT_FUNC_GET_STATS] != NULL)
			{
				const char *func_name = PYAGENT_FUNC_NAMES[PYAGENT_FUNC_GET_STATS];
				PyObject *result = PyObject_CallObject(_agentFuncs[PYAGENT_FUNC_GET_STATS], NULL);
				if (result == NULL)
				{
					ERROR_F("Call to function '%s' in Python agent failed!", func_name);
				}
				else
				{
					// check if list
					if (!PyList_Check(result))
					{
						ERROR_F("Expected a list for return value of function '%s'!", func_name);
					}
					else
					{
						int num_stats = PyList_Size(result);
						for (int i = 0; i < num_stats; i++)
						{
							PyObject *item = PyList_GetItem(result, i);
							if (!PyTuple_Check(item))
							{
								ERROR_F("Expected tuple for item %d in returned list from function '%s'!", i, func_name);
								break;
							}
							PyObject *value = PyTuple_GetItem(item, 1);
							if (write_header)
							{
								PyObject *name = PyTuple_GetItem(item, 0);
								if (!PyUnicode_Check(name))
								{
									ERROR_F("Expected string at first position in tuple %d in returned list from function '%s'!", i, func_name);
									break;
								}
#ifdef ARENA_PYTHON_VERSION_3
								names.push_back(PyUnicode_AsUTF8(name));
#else
								names.push_back(PyString_AsString(name));
#endif
							}
							float fvalue = 0;
							if (PyFloat_Check(value))
							{
								fvalue = PyFloat_AsDouble(value);
							}
							else if (PyLong_Check(value))
							{
								fvalue = (float)PyLong_AsLong(value);
							}
							else
							{
								ERROR_F("Expected float or int at second position in tuple %d in returned list from function '%s'!", i, func_name);
								break;
							}
							values.push_back(fvalue);
						}
					}
					Py_DECREF(result);
				}
			}
			if (write_header)
			{ // write header first
				_csvWriter.writeHeader(names);
			}
			if (values.size() != _csvWriter.getNumCols())
			{
				ERROR_F("Number of metrics changed to %ld (before: %d)!", values.size(), _csvWriter.getNumCols());
			}
			_csvWriter.write(values);
			_csvWriter.flush();
		}
	}
}


// #ifdef USE_ROS
void Arena::rosUpdate(float wait_time = 0.0f)
{
	_videoDisabled = _ros_node_ptr->pause_flag;
	bool any_arrow_key_pressed = false;
	if (_keysPressed[UP] || _keysPressed[DOWN] || _keysPressed[LEFT] || _keysPressed[RIGHT])
	{
		any_arrow_key_pressed = true;
	}
	if (any_arrow_key_pressed)
	{
		int action;
		if (_keysPressed[UP])
		{
			action = Robot::FORWARD;
			if (_keysPressed[LEFT])
			{
				action = Robot::FORWARD_LEFT;
			}
			else if (_keysPressed[RIGHT])
			{
				action = Robot::FORWARD_RIGHT;
			}
		}
		else if (_keysPressed[DOWN])
		{
			action = Robot::BACKWARD;
		}
		else
		{
			if (_keysPressed[LEFT])
			{
				action = Robot::FORWARD_STRONG_LEFT;
			}
			else if (_keysPressed[RIGHT])
			{
				action = Robot::FORWARD_STRONG_RIGHT;
			}
		}
		for (int i = 0; i < _numEnvs; i++)
		{
			Robot::getActionTwist((Robot::Action)action, _actions[i]);
		}
	}
	// we put the wait connection here so that window will not be in blank screen.
	// update: temporary not needed 
	// if (!_ros_node_ptr->m_env_connected && !any_arrow_key_pressed)
	// {
	// 	_ros_node_ptr->waitConnection();
	// 	return;
	// }
	
	RosNode::Status s;

	if (_ros_node_ptr == nullptr && !any_arrow_key_pressed)
		return; // should throw Exception
	if (!any_arrow_key_pressed)
		s = _ros_node_ptr->getActions(_actions, _ros_envs_reset, wait_time);
		
	if(_ros_node_ptr->pause_flag) {
		std::this_thread::sleep_for(std::chrono::microseconds(5));
		return;
		}		
	/* do nothing if not message from all agents received */
	if (!any_arrow_key_pressed && s == RosNode::Status::NOT_ALL_AGENT_MSG_RECEIVED)
	{
		return;
	}
	else if (!any_arrow_key_pressed && s == RosNode::Status::SIM_CLOSE)
	{
		cmdStopTraining(ConsoleParameters(0, NULL));
		INFO("Training done!");
		return;
	}
	else if (!any_arrow_key_pressed && s == RosNode::Status::ENV_RESET)
	{
		for (int i = 0; i < _numEnvs; i++)
		{
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if (_ros_envs_reset[i])
			{
				if (s != Environment::RUNNING)
				{
					_levelResetMeasure.startTime();
					_envs[i].reset(false);
					_levelResetMeasure.endTime();
				}
				else
				{
					//ROS_ERROR_STREAM("Something wrong with the env: " << i);
				}
			}
		}
	}
	else if (any_arrow_key_pressed || s == RosNode::Status::ALL_AGENT_MSG_RECEIVED)
	{
		for (int i = 0; i < _numEnvs; i++)
		{
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if (s != Environment::RUNNING)
			{
				_dones[i] = true;
			}
			else
			{
				_dones[i] = false;
			}
		}
		// threaded simulation step
		_simulationMeasure.startTime();
		for (int i = 0; i < _numThreads; i++)
		{
			_threads[i].step();
		}

		for (int i = 0; i < _numThreads; i++)
		{
			_threads[i].wait_finish();
		}
		_simulationMeasure.endTime();

		// check whether episodes has just ended
		bool episode_over = false;
		int over_env = -1;
		for (int i = 0; i < _numEnvs; i++)
		{
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if (s != Environment::RUNNING && !_dones[i])
			{ // episode is now over and has not been over before
				_dones[i] = true;
				// adding success value to success buffer
				_meanSuccess.push((s == Environment::POSITIVE_END) ? 1 : 0);
				_meanSuccess.calculateMean();
				// adding reward to total reward buffer
				_meanReward.push(_envs[i].getTotalReward());
				_meanReward.calculateMean();


				// stage mode: update number of obstacle
				// if success rate over 0.9, then get into next stage
				// if success rate under 0.7, then directly back to previous stage					
				float cur_success = _meanSuccess.getMean();
				string cur_level = "";
				ros::param::get("level",cur_level);
				string::size_type level = cur_level.find("scenario");
				if((stage_flag) && (level != string::npos)){
					stage_static_obs = 0;
					stage_dynamic_obs = 0;
					episode_buffer = 100;


					if(episode_buffer_flag && episode_buffer_count < episode_buffer){
						++episode_buffer_count;
						INFO_F("Still need %i episode to leave buffer", episode_buffer - episode_buffer_count);
						if(episode_buffer_count == episode_buffer){
							episode_buffer_flag = false;
							// meanSuccess buffer store value in current new stage
						}
					}
					else{
						if(cur_success >= (90.0/100)){
							cur_stage += 1;
							bool flag_static = ros::param::get("/stage_" + to_string(cur_stage) + "/static", stage_static_obs);
							bool flag_dynamic = ros::param::get("/stage_" + to_string(cur_stage) + "/dynamic", stage_dynamic_obs);
							bool has_next_stage = flag_static && flag_dynamic;
							if(has_next_stage){
								_SETTINGS->stage.num_obstacles = stage_static_obs;
								_SETTINGS->stage.num_dynamic_obstacles = stage_dynamic_obs;
								INFO_F("Next stage is stage_%d with num_static = %d, num_dynamic = %d",cur_stage, stage_static_obs, stage_dynamic_obs);
							}
							// if has next stage, update obstacle
							else if(cur_success >= 0.95){
								char time_buffer[32];
								Timer::getTimeString(SDL_GetTicks() - _trainingStartTime, time_buffer, 32);
								INFO_F("Training over. Total training time is %s\n",time_buffer);
								exitApplication();
							}
							// if in final stage and successrafte over 95%, finish training
							else{
								cur_stage -= 1;
							}
							// cur_stage in final stage don't change

							if(episode_buffer_flag == false && has_next_stage){
								episode_buffer_flag = true;
								episode_buffer_count = 0;
								INFO_F("Start to count episode_buffer in next stage: %d ",cur_stage);
							}
							// ready to start buffer

							if(cur_stage > arrived_max_stage) arrived_max_stage = cur_stage;
							// count history arrived maximal stage												
						}
						else if((cur_success <= 0.7 && cur_stage != 1)){
							cur_stage -= 1;
							bool flag_static = ros::param::get("/stage_" + to_string(cur_stage) + "/static", stage_static_obs);
							bool flag_dynamic = ros::param::get("/stage_" + to_string(cur_stage) + "/dynamic", stage_dynamic_obs);
							if(flag_static && flag_dynamic){
								_SETTINGS->stage.num_obstacles = stage_static_obs;
								_SETTINGS->stage.num_dynamic_obstacles = stage_dynamic_obs;
								INFO_F("Back to last stage_%i with num_static = %i, num_dynamic = %i\n",cur_stage, stage_static_obs, stage_dynamic_obs);				
							}			
							// back to last stage if successrate under 0.7
							if(episode_buffer_flag == false){
								episode_buffer_flag = true;
								episode_buffer_count = 0;
								INFO_F("Start to count episode_buffer in previous stage: %d ",cur_stage);
							}		
							// ready to start buffer															
						}
					}				

				}

				// scenerio mode: limit number of episode
				bool scenerio = false;
				int scenerio_count = 20;
				ros::param::get("scenerio/scenerio", scenerio);
				if(scenerio){
					if(_episodeCount > scenerio_count){
						INFO_F("%i evaluations are over",scenerio_count);
						printEpisodeResults(_envs[i].getTotalReward());
						quit();
						// TODO: what should do? store plot/store some information?
					}
				}
				_episodeCount++;
				
				// show results
				printEpisodeResults(_envs[i].getTotalReward());
				if(stage_flag){
					ros::NodeHandle nh;
					nh.setParam("stage/curstage",cur_stage);
					nh.setParam("stage/maxstage",arrived_max_stage);
					INFO_F("  Current Stage is: %i",cur_stage);
					INFO_F("  Arrived Stage is: %i",arrived_max_stage);
				}

				episode_over = true;
				over_env = i;
				// if key pressed reset environment immediately
				if (any_arrow_key_pressed)
				{
					_levelResetMeasure.startTime();
					_envs[i].reset(false);
					_levelResetMeasure.endTime();
				}
			}
		}
		if (episode_over)
		{
			if (_SETTINGS->video.enabled)
			{
				refreshEpisodeCounter();
				refreshRewardCounter();
				_envs[over_env].reset(false);
			}

		}
	}
	
	if (_SETTINGS->video.enabled && !_videoDisabled)
	{
		refreshLevelResetTime();
	}
	// measuring FPS
	_physicsTimer.update(false);
	// if (!any_arrow_key_pressed)
	_ros_node_ptr->publishStates(_dones, _meanReward.getMean(), _meanSuccess.getMean());
}
// #endif

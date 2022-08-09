#include "RosService.hpp"

RosService::RosService(int argc, char **argv)
{
    
    ROS_INFO("in class constructor of ExampleRosClass");
    ros::init(argc, argv, "arena_service");
    _setRosService();

}

void RosService::_setRosService(){
        service_delete = _nh_1.advertiseService("delete_model", &RosService::DeleteModelCallback, this);
        std::cout << "service started" << std::endl << std::endl;

}


bool RosService:: DeleteModelCallback(arena2d_msgs::DeleteModel::Request  &request,
                                   arena2d_msgs::DeleteModel::Response &response){

    // for (int idx_env = 0; idx_env < m_num_envs; idx_env++)
    // {
    //     m_envs[idx_env].getLevel()->clear();
    // }
    // return true;
    ROS_INFO("service callback activated");
    std::cout << "successful" << std::endl;
    response.message = request.name;
    response.success = true;
    return true;
}
#include <ros/ros.h>
#include <arena2d_msgs/DeleteModel.h>
#include <iostream>


class RosService
{
    public:
        RosService(int argc, char **argv);
    
    private:

    ros::NodeHandle _nh_1;
    ros::ServiceServer service_delete;
    void _setRosService();
    bool DeleteModelCallback(arena2d_msgs::DeleteModel::Request  &request,
                                       arena2d_msgs::DeleteModel::Response &response);
                                
};
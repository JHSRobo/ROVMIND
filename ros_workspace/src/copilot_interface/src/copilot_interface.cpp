#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <copilot_interface/copilotControlParamsConfig.h>
#include <std_msgs/UInt16.h>
#include <stdint.h>

//currently test code for developing and testing dynamic reconfigure page

void callback(copilot_interface::copilotControlParamsConfig &config, uint32_t level) {

    ROS_INFO("Reconfigure Request: %f %f %f %s",
             config.l_scale, config.a_scale,
             config.v_scale,
             config.thrustersEnabled?"True":"False");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "copilot_control");

    ros::NodeHandle n;

    dynamic_reconfigure::Server<copilot_interface::copilotControlParamsConfig> server;
    dynamic_reconfigure::Server<copilot_interface::copilotControlParamsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}



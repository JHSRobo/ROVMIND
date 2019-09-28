//Doc mainpage written in horiz_drive.cpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//custom message for holding 4 int32 thruster percents
#include "vector_drive/thrusterPercents.h"


ros::Publisher pub;  //!< Publishes thrusterPercent (-1000 to 1000) message for thruster 5, 6, 7, and 8
ros::Subscriber sub; //!< Subscribes to rov/cmd_vel in order to get command/control vectors for vector drive algorithm   

vector_drive::thrusterPercents thrustPercents; //!< Message being published by pub

//template class for simple functions (breifs in horiz_drive.cpp)
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

template <class T>
T abs(T value){
    if(value < 0)
        value*=-1;
    return value;
}

template <class T>
T max(T value1, T value2){
    if(value1 > value2)
        return value1;
    return value2;
}

template <class T>
T min(T value1, T value2){
    if(value1 < value2)
        return value1;
    return value2;
}

template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}

/**
* @breif updates control percents, updates thruster percents, and publishes the updates thruster percents to the rov/cmd_horizontal_vdrive topic. Currently no vector math is needed due to primitive vertical control system. Control percents are directly multiplied by 1000 for thrusters percents. 
* @param[in] vel vel Input from the joystick, ros_control_interface and ROS Control PID algorithms
*/

void commandVectorCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    //only deals with values pertaining to vertical "vector" drive
    //vertical value
    double linearZ = vel->linear.z;

    double T5 = linearZ*1000;
    double T6 = linearZ*1000;
    double T7 = linearZ*1000;
    double T8 = linearZ*1000;

    thrustPercents.t1 = T5;
    thrustPercents.t2 = T6;
    thrustPercents.t3 = T7;
    thrustPercents.t4 = T8;

    //publish message
    pub.publish(thrustPercents);
}

int main(int argc, char **argv)
{

    //initialize node for horizontal vector drive
    ros::init(argc, argv, "vert_drive");

    ros::NodeHandle n;

    //ROS publisher to send thruster percent to hardware control node for CAN transmission
    pub = n.advertise<vector_drive::thrusterPercents>("rov/cmd_vertical_vdrive", 1);

    //ROS subscriber to get vectors from the joystick control input
    sub = n.subscribe("rov/cmd_vel", 1, commandVectorCallback);


    ros::spin();

    return 0;
}

/**
* @author Michael Equi
* @version 0.1
* @date 8-11-2018
* @mainpage The drive_control node
* @section intro_sec Introduction
* This code contains implementations for converting horizontal and vertical control vectors into individual thruster percents (multiplied by 10 for more accuracy without needing to be stored as doubles) from -1000 to 1000
* @section compile_sec Compilation
* Compile using catkin_make in the ros_workspace directory.
*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//custom message for holding 4 int32 thruster percents
#include "vector_drive/thrusterPercents.h"


ros::Publisher pub;  //!< Publishes thrusterPercent (-1000 to 1000) message for thruster 1, 2, 3, and 4
ros::Subscriber sub; //!< Subscribes to rov/cmd_vel in order to get command/control vectors for vector drive algorithm

vector_drive::thrusterPercents thrustPercents; //!< Message being published by pub

//template classes for simple functions

/**
* @breif constrians value between min and max inclusive. Value is returned by reference.
* @param[in,out] value input to be constrianed
* @param[in] min The minimum value that "value" should be able to be
* @param[in] max The maximum value that "value" should be able to be
*/
template <class T>
void constrain(T &value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
}

/**
* @breif returns the absolute value of value
* @param[in] value
* @return the absolute value of value input
*/
template <class T>
T abs(T value){
    if(value < 0)
        value*=-1;
    return value;
}

/**
* @breif returns the larger number between vlaue1 and value2
* @param[in] value1
* @param[in] value2
* @return The larger of the two values
*/
template <class T>
T max(T value1, T value2){
    if(value1 > value2)
        return value1;
    return value2;
}

/**
* @breif returns the smaller number between value1 and value2
* @param[in] value1
* @param[in] value2
* @return The smaller of the two values
*/
template <class T>
T min(T value1, T value2){
    if(value1 < value2)
        return value1;
    return value2;
}
/**
* @breif returns a number mapped proportioanlly from one range of numbers to another
* @param[in] input Value to be mapped
* @param[in] inMax The maximum value for the range of the input
* @param[in] inMin The minimum value for the range of the input
* @param[in] outMin The minimum value for the range of the output
* @param[in] outMax The maximum value for the range of the output
* @return The input trnslated proportionally from range in to range out
*/
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}

/**
* @breif Takes in linearX, linearY, and anguarX percents (-1 to 1) and translates tham to indvidual thrusters percents from -1000 to 1000. These percents are for thrusters T1, T2, T3, and T4.
* @param[in] linearX Equivalent to the left-right axis of the joystick
* @param[in] linearY Equivalent to the front-back axis of the joystick
* @param[in] angularX Equivalent to the rotational/angular axis of the joystick
* @return Const vector_drive/thrusterPercents message ready to be published to the rov/cmd_horizontal_vdrive topic
*/
const vector_drive::thrusterPercents& vectorMath(const double &linearX, const double &linearY, const double &angularX){
    //if values out of range flag an error
    if(abs(linearX) > 1 || abs(linearY) > 1 || abs(angularX) > 1){
        //ROS_ERROR("cmd_vel value out of range!\nEntering safe mode and disabling thrusters... ");
        ROS_ERROR_STREAM("linearX: " << linearX << "  linearY: " << linearY << "  angularX: " << angularX);
    }

    //inversion, sensitivity, bi-linear threshold are handled in rov_control_interface (drive_control node)
    //deadzone handled by joy package

    //Motor calculations
    //See: https://drive.google.com/file/d/11VF0o0OYVaFGKFvbYtnrmOS0e6yM6IxH/view
    //TODO prevent thrustpercents from going out of 1000 to -1000 range
    double T1 = linearX + linearY + angularX;
    double T2 = -linearX + linearY - angularX;
    double T3 = -linearX - linearY + angularX;
    double T4 = linearX - linearY - angularX;


    //Normalize the values so that no motor outputs over 100% thrust
    double maxMotor = max(max(max(abs(T1), abs(T2)), abs(T3)), abs(T4));
    double maxInput = max(max(abs(linearX), abs(linearY)), abs(angularX));

    if(maxMotor == 0)
        maxMotor = 1;

    T1 *= maxInput / maxMotor;
    T2 *= maxInput / maxMotor;
    T3 *= maxInput / maxMotor;
    T4 *= maxInput / maxMotor;

    //T100 thruster power calculations



    ROS_DEBUG_STREAM("T1: " << thrustPercents.t1 << "  T2: " <<
    	thrustPercents.t2 << "  T3: " << thrustPercents.t3 << "  T4: " << thrustPercents.t4);

    //load thruster values into custom int32 ROS message
    thrustPercents.t1 = T1*1000;
    thrustPercents.t2 = T2*1000;
    thrustPercents.t3 = T3*1000;
    thrustPercents.t4 = T4*1000;


    return thrustPercents;
}

/**
* @breif updates control percents, runs vectorMath, updates thruster percents, and publishes the updates thruster percents to the rov/cmd_horizontal_vdrive topic
* @param[in] vel Input from the joystick, ros_control_interface and ROS Control PID algorithms
*/

void commandVectorCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    //only deals with values pertaining to horizontal vector drive

    //linear (L-R)
    double linearX = vel->linear.x;

    //linear (F-B)
    double linearY = vel->linear.y;

    //angular
    double angularX = vel->angular.x;

    vectorMath(linearX, linearY, angularX);

    //publish message
    pub.publish(thrustPercents);
}

int main(int argc, char **argv)
{
    //initialize node for horizontal vector drive
    ros::init(argc, argv, "horiz_drive");

    ros::NodeHandle n;

    //ROS publisher to send thruster percent to hardware control node for CAN transmission
    pub = n.advertise<vector_drive::thrusterPercents>("rov/cmd_horizontal_vdrive", 1);

    //ROS subscriber to get vectors from the joystick control input
    sub = n.subscribe("rov/cmd_vel", 1, commandVectorCallback);


    ros::spin();

    return 0;
}

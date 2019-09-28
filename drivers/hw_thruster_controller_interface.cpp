#include <ros/ros.h>
#include "vector_drive/thrusterPercents.h"

ros::Subscriber subHorizontals;
ros::Subscriber subVerticals;

int thrusters[8]{0, 0, 0, 0, 0, 0, 0, 0}; //variable for storing thruster values before being sent to ROV

//Handles horizontal thruster updates
void updateHorizontalsCallback(const vector_drive::thrusterPercents::ConstPtr& horizontals){
    thrusters[0] = horizontals.t1;
    thrusters[1] = horizontals.t2;
    thrusters[2] = horizontals.t3;
    thrusters[3] = horizontals.t4;
}

//Handles vertical thruster updates
void updateVerticalsCallback(const vector_drive::thrusterPercents::ConstPtr& verticals){
    thrusters[4] = verticals.t1;
    thrusters[5] = verticals.t2;
    thrusters[6] = verticals.t3;
    thrusters[7] = verticals.t4;
}

//Set the rate that the serial connection with the thruster controller updates and message are published if needed
ros::Rate loop_rate(20); //20Hz

int main(int argc, char **argv){
    ros::init(argc, argv, "hw_thruster_controller_interface");

    ros::nodeHandle n;

    subHorizontals = n.subscribe("rov/cmd_horizontal_vdrive", 1, updateHorizontalsCallback);
    subVerticals = n.subscribe("rov/cmd_vertical_vdrive", 1, updateVerticalsCallback);

    while (ros::ok()){
        ros::spinOnce(); //run through subscriber callbacks



        loop_rate.sleep(); //maintain 20Hz frequency
    }
}
#include <ros.h>
#include <std_msgs/String.h>
#include <thrusterPercents.h>

#include <Servo.h> 

//decalre 8 thrusters (T7 and T8 not used in this implemntation due to arudino uno having too few pins)
Servo T1, T2, T3, T4, T5, T6, T7, T8;


//declare ROS node handle
ros::NodeHandle  nh;

//Update the rpi on the status of the thruster -> not yet fully implemented
std_msgs::String str_msg; //For holding the status_message until it is sent by the publisher

ros::Publisher thruster_status("thruster_controller_status", &str_msg);

char statusMsg[20] = ""; //up to 19 characters in the status message


//handle messages recieved from rpi
void horizontal_vdrive_messageCb( const vector_drive::thrusterPercents& thrusterValuePacket){
  T1.writeMicroseconds(map(thrusterValuePacket.t1, -1000, 1000, 1100, 1900));
  T2.writeMicroseconds(map(thrusterValuePacket.t2, -1000, 1000, 1100, 1900));
  T3.writeMicroseconds(map(thrusterValuePacket.t3, -1000, 1000, 1100, 1900));
  T4.writeMicroseconds(map(thrusterValuePacket.t4, -1000, 1000, 1100, 1900));

  str_msg.data = "Horizontal Thrusters Updated";
  thruster_status.publish( &str_msg );
}

void vertical_vdrive_messageCb( const vector_drive::thrusterPercents& thrusterValuePacket){
  T5.writeMicroseconds(map(thrusterValuePacket.t1, -1000, 1000, 1100, 1900));
  T6.writeMicroseconds(map(thrusterValuePacket.t2, -1000, 1000, 1100, 1900));

  str_msg.data = "Vertical Thrusters Updated";
  thruster_status.publish( &str_msg );
}


ros::Subscriber<vector_drive::thrusterPercents> sub1("rov/cmd_horizontal_vdrive", horizontal_vdrive_messageCb );

ros::Subscriber<vector_drive::thrusterPercents> sub2("rov/cmd_vertical_vdrive", vertical_vdrive_messageCb );


void setup()
{
  nh.initNode();
  nh.advertise(thruster_status);
  
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  //setup hardware pwm pins
  T1.attach(3);
  T2.attach(5);
  T3.attach(6);
  T4.attach(9);
  T5.attach(10);
  T6.attach(11);
  
  //setup thrusters to zero point to set up ESCs
  T1.writeMicroseconds(1500);
  T2.writeMicroseconds(1500);
  T3.writeMicroseconds(1500);
  T4.writeMicroseconds(1500);
  T5.writeMicroseconds(1500);
  T6.writeMicroseconds(1500);

  delay(100);

  str_msg.data = "Thruster Controller Enabled";
  thruster_status.publish( &str_msg );
}

void loop()
{
  nh.spinOnce();
  delay(1);
}


#!/usr/bin/env sh

#for setting up ROS enviroment for  launch file accross multiple machines

. /opt/ros/kinetic/setup.sh
source ~/Desktop/ROV_Test_Bench/ros_workspace/devel/setup.bash
exec "$@"

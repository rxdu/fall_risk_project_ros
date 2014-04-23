#include "ros/ros.h"

#include <remote_command_server/RemoteCmdSrv.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_command_server");


//  ros::Subscriber sub = nh.subscribe("/sensors/luminosity", 1000, getLuminosity);
//// Add new subscribers and callback functions for other sensors topic
//  ros::ServiceServer service = nh.advertiseService("checklist_status", getChecklistStatus);
  ROS_INFO("Ready to provide checklist information:");
  ros::spin();

  return 0;
}

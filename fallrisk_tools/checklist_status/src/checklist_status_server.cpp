#include "ros/ros.h"
#include "checklist_status/ChecklistStatusSrv.h"

bool getChecklistStatus(checklist_status::ChecklistStatusSrv::Request  &req,
         checklist_status::ChecklistStatusSrv::Response &res)
{
//  res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
//  return true;
    res.luminosity=20;
    ROS_INFO("Checklist update responded!");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checklist_status_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("checklist_status", getChecklistStatus);
  ROS_INFO("Ready to provide checklist information:");
  ros::spin();

  return 0;
}

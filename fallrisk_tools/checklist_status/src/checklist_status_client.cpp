#include "ros/ros.h"
#include "checklist_status/ChecklistStatusSrv.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checklist_status_client");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<checklist_status::ChecklistStatusSrv>("checklist_status");

  checklist_status::ChecklistStatusSrv srv;

  if (client.call(srv))
    {
      ROS_INFO("Luminosity: %f", srv.response.luminosity);
    }
    else
    {
      ROS_ERROR("Failed to call service checklist_status");
      return 1;
    }

  return 0;
}

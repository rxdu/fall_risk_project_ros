#include "ros/ros.h"
#include <remote_command_server/RemoteCmdSrv.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_command_client");
  ros::NodeHandle nh_;

  ros::ServiceClient client = nh_.serviceClient<remote_command_server::RemoteCmdSrv>("remote_command");

  remote_command_server::RemoteCmdSrv remoteCmdSrv;

  remoteCmdSrv.request.cmd_name=remoteCmdSrv.request.CMD_TELEPRESENCE;
  remoteCmdSrv.request.cmd_action=remoteCmdSrv.request.STOP;

  if(client.call(remoteCmdSrv))
  {
      if(remoteCmdSrv.response.cmd_status)
          ROS_INFO("SUCCESS");
      else
          ROS_INFO("FAILURE");
  }
  else
  {
      ROS_ERROR("Failed to call service remote_command");
  }

  return 0;
}

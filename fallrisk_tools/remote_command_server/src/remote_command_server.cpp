#include <stdlib.h>
#include <ros/ros.h>
#include <remote_command_server/RemoteCmdSrv.h>

int runSysCommand(const char* cmd);

bool executeCommand(remote_command_server::RemoteCmdSrv::Request &req, remote_command_server::RemoteCmdSrv::Response &res)
{
    if(req.cmd_name == req.CMD_AMCL)
    {
        if(req.cmd_action == req.START)
        {
            runSysCommand("roslaunch uvc_camera_image uvc_camera_start.launch &");

            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("amcl start");
        }
        else if(req.cmd_action == req.STOP)
        {
            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("amcl stop");
        }
    }
    else if(req.cmd_name == req.CMD_GMAPPING)
    {
        if(req.cmd_action == req.START)
        {
            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("gmapping start");
        }
        else if(req.cmd_action == req.STOP)
        {
            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("gmapping stop");
        }
    }

    return true;
}

int runSysCommand(const char* cmd)
{
    int i=0;
    int j=1;
    int procID=0;

    if (system(NULL))
        ROS_INFO("Commands can be accpeted:");
    else
        ROS_INFO("System is not ready to excute commands!");

    i=system (cmd);
    j=system ("$!");
    printf ("The value returned was: %d.\n",i);

    return i;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "checklist_status_server");
  ros::NodeHandle nh_;

  ros::ServiceServer cmd_server = nh_.advertiseService("remote_command",executeCommand);
  ROS_INFO("Ready to respond to remote command:");
  ros::spin();

  return 0;
}

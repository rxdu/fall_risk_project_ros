#include <stdlib.h>
#include <ros/ros.h>
#include <remote_command_server/RemoteCmdSrv.h>
#include <QObject>
#include <QProcess>
#include <QString>

int runSysCommand(const char* cmd);

bool executeCommand(remote_command_server::RemoteCmdSrv::Request &req, remote_command_server::RemoteCmdSrv::Response &res)
{
        QObject* parent = new QObject();
        QString program = "gedit";
        QStringList arguments;
        arguments << "~/test.txt"<<"anotherargument"<<"OneMore";

         QProcess* myProcess = new QProcess(parent);

    if(req.cmd_name == req.CMD_AMCL)
    {
        if(req.cmd_action == req.START)
        {
          myProcess->start(program);

            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("Process started with PID :%d",myProcess->pid());
            ROS_INFO("Killing the process");

            myProcess->kill();
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "checklist_status_server");
  ros::NodeHandle nh_;

  ros::ServiceServer cmd_server = nh_.advertiseService("remote_command",executeCommand);
  ROS_INFO("Ready to respond to remote command:");
  ros::spin();

  return 0;
}

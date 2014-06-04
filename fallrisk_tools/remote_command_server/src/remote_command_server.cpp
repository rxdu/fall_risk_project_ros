#include <stdlib.h>
#include <ros/ros.h>
#include <remote_command_server/RemoteCmdSrv.h>
#include <QObject>
#include <QProcess>
#include <QString>

QProcess* navigationProcess = new QProcess(new QObject());
QProcess* telepresenceProcess = new QProcess(new QObject());

int runSysCommand(const char* cmd);

bool executeCommand(remote_command_server::RemoteCmdSrv::Request &req, remote_command_server::RemoteCmdSrv::Response &res)
{
    //    if(req.cmd_name == req.CMD_AMCL)
    //    {
    //        if(req.cmd_action == req.START)
    //        {
    ////            QString program = "roslaunch";
    ////            QStringList arguments;
    ////            arguments << "uvc_camera_image"<<"uvc_camera_start.launch";

    ////            for(int i=0; i<arguments.size();i++) {
    ////                std::cout<<arguments.at(i).toStdString()<<std::endl;
    ////            }

    ////            std::cout<<program.toStdString()<<std::endl;
    ////            myProcess->start(program, arguments);

    ////            ROS_INFO("STATE:%d",myProcess->state());
    //            ROS_INFO("amcl start");
    ////            ROS_INFO("Process started with PID :%d",myProcess->pid());

    //            res.cmd_status = res.CMD_SUCCESS;
    ////            res.cmd_PID = myProcess->pid();
    //        }
    //        else if(req.cmd_action == req.STOP)
    //        {
    ////            myProcess->terminate();

    ////            ROS_INFO("STATE:%d",myProcess->state());
    ////            ROS_INFO("Killing process PID: %d",myProcess->pid());
    //            ROS_INFO("amcl stop");

    //            res.cmd_status = res.CMD_SUCCESS;
    //        }
    //    }
    //    else if(req.cmd_name == req.CMD_GMAPPING)
    //    {
    //        if(req.cmd_action == req.START)
    //        {
    ////            QString program = "roslaunch";
    ////            QStringList arguments;
    ////            arguments << "uvc_camera_image"<<"uvc_camera_start.launch";

    ////            for(int i=0; i<arguments.size();i++) {
    ////                std::cout<<arguments.at(i).toStdString()<<std::endl;
    ////            }

    ////            std::cout<<program.toStdString()<<std::endl;
    ////            myProcess->start(program, arguments);

    ////            ROS_INFO("STATE:%d",myProcess->state());
    //            ROS_INFO("gmapping start");
    ////            ROS_INFO("Process started with PID :%d",myProcess->pid());

    ////            res.cmd_status = res.CMD_SUCCESS;
    ////            res.cmd_PID = myProcess->pid();
    //        }
    //        else if(req.cmd_action == req.STOP)
    //        {
    ////            myProcess->terminate();

    ////            ROS_INFO("STATE:%d",myProcess->state());
    ////            ROS_INFO("Killing process PID: %d",myProcess->pid());
    //            ROS_INFO("gmapping stop");
    //        }
    //    }


    ROS_INFO("Navigation Process State:%d",navigationProcess->state());
    ROS_INFO("Telepresence Process State:%d",telepresenceProcess->state());
    if (req.cmd_name == req.CMD_TELEPRESENCE){
        if(telepresenceProcess->state() == 0 && req.cmd_action == req.START)
        {
            QString program = "roslaunch";
            QStringList arguments;
            arguments << "fallrisk_audio"<<"audio_turtlebot.launch";

            telepresenceProcess->start(program, arguments);

            ROS_INFO("STATE:%d",telepresenceProcess->state());

            telepresenceProcess->waitForStarted(3000);

            if(telepresenceProcess->state() == 2)
            {
                res.cmd_status = res.CMD_SUCCESS;
                res.cmd_PID = telepresenceProcess->pid();
                ROS_INFO("Telepresence started");
            }
            else
            {
                telepresenceProcess->terminate();
                res.cmd_status = res.CMD_FAILURE;
                ROS_INFO("Telepresence failed to start");
            }
        }
        else
        {
            telepresenceProcess->terminate();
            telepresenceProcess->waitForFinished(3000);
            res.cmd_status = res.CMD_SUCCESS;
            ROS_INFO("Telepresence Stopped");

           }
    }
    else {
        if(navigationProcess->state() != 0){
            navigationProcess->terminate();
            navigationProcess->waitForFinished(3000);
        }

        if(req.cmd_name == req.CMD_AMCL)
        {
            QString program = "roslaunch";
            QStringList arguments;
            arguments << "fallrisk_turtlebot_navigation"<<"amcl_navigtaion.launch";

            navigationProcess->start(program, arguments);

            ROS_INFO("STATE:%d",navigationProcess->state());

            navigationProcess->waitForStarted(3000);

            if(navigationProcess->state() == 2)
            {
                res.cmd_status = res.CMD_SUCCESS;
                res.cmd_PID = navigationProcess->pid();
                ROS_INFO("amcl started");
            }
            else
            {
                navigationProcess->terminate();
                navigationProcess->waitForFinished(3000);
                res.cmd_status = res.CMD_FAILURE;
                ROS_INFO("amcl failed to start");
            }
        }
        else if(req.cmd_name == req.CMD_GMAPPING)
        {
            QString program = "roslaunch";
            QStringList arguments;
            arguments << "fallrisk_turtlebot_navigation"<<"turtlebot_gmapping.launch";

            navigationProcess->start(program, arguments);

            navigationProcess->waitForStarted(3000);

            ROS_INFO("STATE:%d",navigationProcess->state());
            if(navigationProcess->state() == 2)
            {
                res.cmd_status = res.CMD_SUCCESS;
                res.cmd_PID = navigationProcess->pid();
                ROS_INFO("gmapping started");
            }
            else
            {
                navigationProcess->terminate();
                navigationProcess->waitForFinished(3000);
                res.cmd_status = res.CMD_FAILURE;
                ROS_INFO("gmapping failed to start");
            }
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

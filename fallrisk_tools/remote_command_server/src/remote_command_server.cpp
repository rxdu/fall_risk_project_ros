#include <stdlib.h>
#include <ros/ros.h>
#include <remote_command_server/RemoteCmdSrv.h>
#include <QObject>
#include <QProcess>
#include <QString>
#include <std_srvs/Empty.h>

ros::NodeHandle nh_;

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
            ROS_INFO("%d",req.cmd_name);

            if(req.cmd_name==req.CMD_AMCL){
            //Entering this condition means that user switched out of gmapping to AMCL.

                //@TODO: Prompt user to save map




                //Find the path to maps folder on turtlebot
                QProcess* findProcess = new QProcess(new QObject());
                QStringList findArguments;
                findArguments<<"find"<<"fallrisk_turtlebot_navigation";
                findProcess->start("rospack",findArguments);
                findProcess->waitForFinished();
                QString pathToMaps(findProcess->readAllStandardOutput());
                pathToMaps.remove(QChar('\n'));
                pathToMaps = pathToMaps.append("/maps/default");
                ROS_INFO_STREAM(pathToMaps.toStdString());
                delete findProcess;

                //run mapsaver command to save existing map as default.pm in the maps directory
                QProcess* mapsaver = new QProcess(new QObject());
                QString program = "rosrun";
                QStringList arguments;
                arguments << "map_server"<<"map_saver"<<"-f"<<pathToMaps;
                ROS_INFO("Saving Map");
                mapsaver->start(program,arguments);
                bool saved = mapsaver->waitForFinished(10000);
                if(saved)
                    ROS_INFO("Map saved successfully");
                else
                    ROS_INFO("Failed to save the map");
                delete mapsaver;
            }

            navigationProcess->terminate();
            navigationProcess->waitForFinished(3000);
        }

        if(req.cmd_name == req.CMD_AMCL)
        {
            //reset the octomap
            //two ways to call a service in cpp
//            std_msgs::Empty emptySrv;
//            if (ros::service::call("/octomap_server/reset", emptySrv))
//            {
//                ROS_INFO("Octomap is reset successfully!");
//            }
//            else
//            {
//                ROS_INFO("Failed to reset octomap!");
//            }

            ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");
            std_srvs::Empty emptySrv;

            if (client.call(emptySrv))
            {
                ROS_INFO("Octomap is reset successfully!");
            }
            else
            {
                ROS_INFO("Failed to reset octomap!");
            }

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
//    ros::NodeHandle nh_;

    ros::ServiceServer cmd_server = nh_.advertiseService("remote_command",executeCommand);
    ROS_INFO("Ready to respond to remote command:");
    ros::spin();

    return 0;
}

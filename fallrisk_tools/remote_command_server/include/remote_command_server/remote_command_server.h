#ifndef REMOTE_COMMAND_SERVER_H
#define REMOTE_COMMAND_SERVER_H

#include <ros/ros.h>

class RemoteCommandServer{

public:
    RemoteCommandServer();

private:
    ros::NodeHandle nh;

};



#endif //REMOTE_COMMAND_SERVER_H

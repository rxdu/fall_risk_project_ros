/*
#include <ros/ros.h>
#include <tf/transform_listener.h>

double dist;
double angular;

void subCB(const geometry_msgs::Twist& msg)
{
    ROS_INFO("%f,%f",msg.linear.x,msg.linear.y);
    double linear = msg.linear.x;
    double time = 0.5;

    dist = linear * time;
    angular = msg.angular.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rviz_gui_navigation_goal");

    ros::NodeHandle nh;
    ros::Publisher direction_pub, goal_pub;
    ros::Subscriber gui_sub;

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    direction_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal",100);
    gui_sub = nh.subscribe("/some_topic", 1, subCB);

    double   xCurrent = 0.0;
    double   yCurrent = 0.0;
    double  zCurrent = 0.0;
    double yawCurrent = 0.0;
    geometry_msgs::Quaternion quatMsg;
    geometry_msgs::Twist cmd;
    geometry_msgs::PoseStamped goal; // Goal position message of geometry_msgs::PoseStamped type


    while (nh.ok())

    {
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/base_link", "/map",
                                      ros::Time(0),ros::Duration(10.0));
            listener.lookupTransform("/base_link", "/map",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());

        }

        xCurrent = transform.getOrigin().x();
        yCurrent = transform.getOrigin().y();
        zCurrent = transform.getOrigin().z();

//        ROS_INFO("%f,%f",xCurrent,yCurrent);
        tf::Quaternion quat = transform.getRotation();
        tf::quaternionTFToMsg(quat, quatMsg);
        yawCurrent = tf::getYaw(quatMsg);


        (goal.header).frame_id = "map";
        (goal.header).stamp = ros::Time::now();
        (goal.pose).position.x = xCurrent + (dist) * cos(yawCurrent);
        (goal.pose).position.y = yCurrent + (dist) * sin(yawCurrent);
        (goal.pose).position.z = zCurrent;
        (goal.pose).orientation = quatMsg;
        (cmd.linear).x = 0;
        (cmd.linear).y = 0;
        (cmd.linear).z = 0;
        (cmd.angular).x = 0;
        (cmd.angular).y = 0;
        (cmd.angular).z = angular;
        direction_pub.publish(cmd);
        goal_pub.publish(goal);
        rate.sleep();
    }
    return 0;
}
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class rviz_gui_handler
{
private:

    geometry_msgs::Twist cmd;
    geometry_msgs::PoseStamped goal; // Goal position message of geometry_msgs::PoseStamped type

    ros::NodeHandle nh;
    ros::Publisher direction_pub, goal_pub;
    ros::Subscriber gui_sub;

    tf::TransformListener listener; // Create listener object

    double dist;
    double angular;
    double xCurrent;
    double yCurrent;
    double zCurrent;
    double yawCurrent;
    geometry_msgs::Quaternion quatMsg;

public:

    ~rviz_gui_handler();
    void init();
    void subCB(geometry_msgs::TwistConstPtr msg);
    void getCurrentPosition();
    void setGoalPosition();
    void publishGoal();
};

void rviz_gui_handler::init()
{
    direction_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 8);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal",8);
    gui_sub = nh.subscribe("/some_topic", 8, &rviz_gui_handler::subCB,this);

    dist = 0.0;
    angular = 0.0;
    xCurrent = 0.0;
    yCurrent = 0.0;
    zCurrent = 0.0;
    yawCurrent = 0.0;

}

rviz_gui_handler::~rviz_gui_handler()
{

}

void rviz_gui_handler::subCB(geometry_msgs::TwistConstPtr msg)
{
    ROS_INFO("Inside callback Function");
    double linear = msg->linear.x;
    double time = 0.5;

    dist = linear * time;
    angular = msg->angular.z;
}

void rviz_gui_handler::getCurrentPosition()
{
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("/base_link", "/map",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/base_link", "/map",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    xCurrent = transform.getOrigin().x();
    yCurrent = transform.getOrigin().y();
    zCurrent = transform.getOrigin().z();
    tf::Quaternion quat = transform.getRotation();
    tf::quaternionTFToMsg(quat, quatMsg);
    yawCurrent = tf::getYaw(quatMsg);

//    ROS_INFO("%f,%f",xCurrent,yCurrent);
}

void rviz_gui_handler::setGoalPosition()
{

    (goal.header).frame_id = "map";
    (goal.header).stamp = ros::Time::now();
    (goal.pose).position.x = xCurrent + (dist) * cos(yawCurrent);
    (goal.pose).position.y = yCurrent + (dist) * sin(yawCurrent);
    (goal.pose).position.z = zCurrent;
    (goal.pose).orientation = quatMsg;

    (cmd.linear).x = 0;
    (cmd.linear).y = 0;
    (cmd.linear).z = 0;

    (cmd.angular).x = 0;
    (cmd.angular).y = 0;
    (cmd.angular).z = angular;
}

void rviz_gui_handler::publishGoal()
{
    direction_pub.publish(cmd);
    goal_pub.publish(goal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_gui_handler");

    rviz_gui_handler handler;
    handler.init();

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        handler.getCurrentPosition();
        handler.setGoalPosition();
        handler.publishGoal();

        ros::spinOnce();
        rate.sleep();
    }

    return(0);
}

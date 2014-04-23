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
    void subCB(const geometry_msgs::Twist& msg);
    void getCurrentPosition();
    void setGoalPosition();
    void publishGoal();
  };

  void rviz_gui_handler::init()
      {
        direction_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 100);
        goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal",100);

        dist = 0.0;
        angular = 0.0;
        xCurrent = 0.0;
        yCurrent = 0.0;
        zCurrent = 0.0;
        yawCurrent = 0.0;

        ros::NodeHandle n_private("~");
      }

      rviz_gui_handler::~rviz_gui_handler()
      {

      }

  void rviz_gui_handler::subCB(const geometry_msgs::Twist& msg)
  {
	  double linear = msg.linear.x;
	  double time = 0.5;

	  dist = linear * time;
	  angular = msg.angular.z;
  }

  void rviz_gui_handler::getCurrentPosition()
  {
	  tf::StampedTransform transform;
	  listener.waitForTransform("map", "base_link",ros::Time::now(), ros::Duration(30.0));
	  listener.lookupTransform("map", "base_link",ros::Time(0), transform);

	  xCurrent = transform.getOrigin().x();
	  yCurrent = transform.getOrigin().y();
	  zCurrent = transform.getOrigin().z();
	  tf::Quaternion quat = transform.getRotation();
	  tf::quaternionTFToMsg(quat, quatMsg);
	  yawCurrent = tf::getYaw(quatMsg);
  }

  void rviz_gui_handler::setGoalPosition()
  {
	  gui_sub = nh.subscribe("some_topic", 10, &rviz_gui_handler::subCB,this);

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

    while(ros::ok())
    {
    	handler.getCurrentPosition();
    	handler.setGoalPosition();
    	handler.publishGoal();
    }

    return(0);
  }

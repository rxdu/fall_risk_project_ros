#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>


  #define KEYCODE_A 0x61
  #define KEYCODE_D 0x64
  #define KEYCODE_S 0x73
  #define KEYCODE_W 0x77

  class TeleopKeyboard
  {
    private:
    geometry_msgs::Twist cmd;
    geometry_msgs::PoseStamped goal; // Goal position message of geometry_msgs::PoseStamped type

    ros::NodeHandle nh;
    ros::Publisher direction_pub, goal_pub;

    tf::TransformListener listener; // Create listener object

    public:
    void init()
    {


      //Clear out our cmd - these values are roundabout initials

      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;

      direction_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 100);
      goal_pub = nh.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal",100);

      ros::NodeHandle n_private("~");
    }

    ~TeleopKeyboard()   { }
    void keyboardLoop();
  };

  int kfd = 0;
  struct termios cooked, raw;

  void quit(int sig)
  {
    tcsetattr(kfd, TCSANOW, &cooked);
   exit(0);
  }

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "teleop_new");

    TeleopKeyboard tele;
    tele.init();

    signal(SIGINT,quit);

    tele.keyboardLoop();

    return(0);
  }

  void TeleopKeyboard::keyboardLoop()
  {
    char c;
   bool dirty=false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'WS' to forward/back");
    puts("Use 'AD' to left/right");

    for(;;)
    {
      tf::StampedTransform transform;
      listener.waitForTransform("map", "base_link",ros::Time::now(), ros::Duration(30.0));
      listener.lookupTransform("map", "base_link",ros::Time(0), transform);

      float xCurrent = transform.getOrigin().x();
      float yCurrent = transform.getOrigin().y();
      float zCurrent = transform.getOrigin().z();
      tf::Quaternion quat = transform.getRotation();
      geometry_msgs::Quaternion quatMsg;
      tf::quaternionTFToMsg(quat, quatMsg);
      double yaw = tf::getYaw(quatMsg);

      float dist = 0.3;

      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      switch(c)
      {
        // Moving forward
      case KEYCODE_W:
    puts("move forward");
        goal.header.frame_id = "map";
	goal.header.stamp = ros::Time::now();
	goal.pose.position.x = xCurrent + dist*cos(yaw);
	goal.pose.position.y = yCurrent + dist*sin(yaw);
	goal.pose.position.z = zCurrent;
	goal.pose.orientation = quatMsg;
        dirty = true;
        break;
	// Moving backward
      case KEYCODE_S:
    puts("move backward");
        goal.header.frame_id = "map";
	goal.header.stamp = ros::Time::now();
	goal.pose.position.x = xCurrent - dist*cos(yaw);
	goal.pose.position.y = yCurrent - dist*sin(yaw);
	goal.pose.position.z = zCurrent;
	goal.pose.orientation = quatMsg;
        dirty = true;
        break;
	// Turn left
      case KEYCODE_A:
    puts("turn left");
        cmd.angular.z = 0.7;
        dirty = true;
        break;
	// Turn right
      case KEYCODE_D:
    puts("turn right");
        cmd.angular.z = -0.7;
        dirty = true;
        break; 
      }


      if (dirty == true)
      {
        direction_pub.publish(cmd);
	goal_pub.publish(goal);
      }

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

    }
  }

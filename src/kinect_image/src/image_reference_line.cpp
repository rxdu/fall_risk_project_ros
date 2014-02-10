#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

//static const char WINDOW[] = "Image window";

class ImageRefLine
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber center_dist_sub_;
  image_transport::Publisher image_pub_;

  float center_dist;

public:
  ImageRefLine()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw_center_dist", 1, &ImageRefLine::imageCb, this);
    image_pub_ = it_.advertise("/camera/rgb/image_raw_ref_line", 1);
  }

  ~ImageRefLine()
  {
    //cv::destroyWindow(WINDOW);
  }

  void distCb(const std_msgs::Float32::ConstPtr& msg)
  {
      center_dist=msg->data;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
        cv::line(cv_ptr->image,cv::Point(cv_ptr->image.cols/5,cv_ptr->image.rows), cv::Point(cv_ptr->image.cols/3,cv_ptr->image.rows/2),cv::Scalar(255,0,0),1,8,0);
        cv::line(cv_ptr->image,cv::Point(cv_ptr->image.cols/5*4,cv_ptr->image.rows), cv::Point(cv_ptr->image.cols/3*2,cv_ptr->image.rows/2),cv::Scalar(255,0,0),1,8,0);
    }

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_ref_line");
  ImageRefLine irf;
  ros::spin();
  return 0;
}


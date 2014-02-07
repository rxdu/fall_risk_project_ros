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

class ImageCenterDist
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber center_dist_sub_;
  image_transport::Publisher image_pub_;
  
  float center_dist;

public:
  ImageCenterDist()
    : it_(nh_)
  {    
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageCenterDist::imageCb, this);
    image_pub_ = it_.advertise("/camera/rgb/image_raw_center_dist", 1);

    center_dist_sub_ = nh_.subscribe("/distance/image_center_dist", 1, &ImageCenterDist::distCb, this);

    //cv::namedWindow(WINDOW);
  }

  ~ImageCenterDist()
  {
    //cv::destroyWindow(WINDOW);
  }

  void distCb(const std_msgs::Float32::ConstPtr& msg)
  {
	  center_dist=msg->data;
	  //cout<<"center_dist="<<center_dist<<endl;
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
    	//std::string txtLabel = str( boost::format("%d") % cv_ptr->image.at<float>(320,240));
    	std::string txtLabel = str( boost::format("%d") % center_dist);

    	cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2+30), 10, CV_RGB(255,0,0));
	//cv::circle(cv_ptr->image, cv::Point(320,240), 15, CV_RGB(255,0,0)); 

        //cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.rows/2,cv_ptr->image.cols/2), 20, CV_RGB(255,0,0));
        
	//cv::circle(cv_ptr->image, cv::Point(640, 480), 10, CV_RGB(255,0,0));
    	cv::putText(cv_ptr->image, txtLabel, cv::Point(cv_ptr->image.cols/2+10, cv_ptr->image.rows/2-5), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0) );
    }

    //cv::imshow(WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_center_dist");
  ImageCenterDist icd;
  ros::spin();
  return 0;
}


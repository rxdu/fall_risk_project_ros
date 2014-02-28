// ROS specific includes

// PCL specific includes

// User defined includes
#include "../include/kinect_depth_common.h"

ros::Publisher pub_plane;

void cloud_cb (const PCLPointXYZPtr& input)
{
//    sensor_msgs::PointCloud2 output;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//    pcl_conversions::toPCL(*input, input_pcl);
//    pcl::fromPCLPointCloud2(input_pcl, *cloud);
////    pcl::fromROSMsg (*input, *cloud);   //deprecated method to do conversion

//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;

    //do something here

//    std::cerr << "Cloud after filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;

//    pcl::toPCLPointCloud2(*cloud,output_pcl);
//    pcl_conversions::fromPCL(output_pcl,output);
////    pcl::toROSMsg(*cloud_filtered,output);   //deprecated method to do conversion

//    // Publish the data
//    pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_plane_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points/filtered_pcl", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_plane = nh.advertise<PCLPointXYZ> ("/camera/depth/points/plane", 1);

  // Spin
  ros::spin ();
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{       
    pcl::PCLPointCloud2::Ptr input_pcl;
    pcl::PCLPointCloud2::Ptr output_pcl;
    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

    pcl_conversions::toPCL(*input, *input_pcl);
    pcl::fromPCLPointCloud2(*input_pcl, *cloud_original);

    // Perform the actual filtering
//    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
//    sor.setInputCloud (input);
//    sor.setLeafSize (0.01, 0.01, 0.01);
//    sor.filter (cloud_filtered);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_original);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

//    // Publish the data
    pcl::toPCLPointCloud2(*cloud_filtered,*output_pcl);
    //pcl::toPCLPointCloud2(*cloud_original,*output_pcl);
    pcl_conversions::fromPCL(*output_pcl,output);
    pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/filtered_points", 1);

  // Spin
  ros::spin ();
}

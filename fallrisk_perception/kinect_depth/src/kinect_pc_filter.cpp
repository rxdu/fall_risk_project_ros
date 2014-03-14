
// ROS specific includes

// PCL specific includes
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// User defined includes
#include "../include/kinect_depth_common.h"

//ros::Publisher pub;
ros::Publisher pub_pcl;

void applyPassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void applyStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 input_pcl;
    pcl::PCLPointCloud2 output_pcl;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(*input, input_pcl);
    pcl::fromPCLPointCloud2(input_pcl, *cloud);
//    pcl::fromROSMsg (*input, *cloud);   //deprecated method to do conversion

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // PassThrough Filter
    //applyPassFilter(cloud);

    // StatisticalOutlierRemoval Filter
    applyStatisticalOutlierRemoval(cloud);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::toPCLPointCloud2(*cloud,output_pcl);
    pcl_conversions::fromPCL(output_pcl,output);
//    pcl::toROSMsg(*cloud_filtered,output);   //deprecated method to do conversion

    // Publish the data
    //pub.publish (output);
    pub_pcl.publish(cloud);

}

void applyPassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //TODO filter limits need to be adjusted
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-15, 15);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-15, 15);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-15, 15);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);
}

void applyStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_pcl_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/points/filtered", 1);
  pub_pcl = nh.advertise<PCLPointXYZ> ("/camera/depth/points/filtered_pcl", 1);

  // Spin
  ros::spin ();
}

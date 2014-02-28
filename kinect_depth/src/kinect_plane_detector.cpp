
#include <iostream>
#include <vector>

// ROS specific includes

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


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


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input;
//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    //do something here
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int nr_points = (int) cloud->points.size ();

    // While 30% of the original cloud is still there
//    while (cloud->points.size () > 0.3 * nr_points)
//    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
 //           break;
        }
        else
        {
            std::cerr << "segment successfully" << std::endl;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
//    }

    PCLPointXYZRGB color_cloud;
    copyPointCloud(*cloud_p, color_cloud);

    for (size_t i = 0; i < color_cloud.points.size(); i++) {
        // pack r/g/b into rgb
        uint8_t r = 0, g = 0, b = 255;    // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

        color_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }


//    std::cerr << "Cloud after filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;

//    pcl::toPCLPointCloud2(*cloud,output_pcl);
//    pcl_conversions::fromPCL(output_pcl,output);
////    pcl::toROSMsg(*cloud_filtered,output);   //deprecated method to do conversion

//    // Publish the data
    //pub_plane.publish (*cloud_p);
    pub_plane.publish (color_cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_plane_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points/filtered_pcl", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_plane = nh.advertise<PCLPointXYZRGB> ("/camera/depth/points/plane", 1);

  // Spin
  ros::spin ();
}

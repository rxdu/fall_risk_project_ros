
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <iterator>

// ROS specific includes

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


// User defined includes
#include "../include/kinect_depth_common.h"

using namespace std;

ros::Publisher pub_plane;
void applyPassFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


void cloud_cb (const PCLPointXYZPtr& input)
{
//    sensor_msgs::PointCloud2 output;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//    pcl_conversions::toPCL(*input, input_pcl);
//    pcl::fromPCLPointCloud2(input_pcl, *cloud);
////    pcl::fromROSMsg (*input, *cloud);   //deprecated method to do conversion


    PCLPointXYZRGB cluster, color_cloud;
    PCLPointXYZ floor;
    std::vector<PCLPointXYZ> planes;
    PCLPointXYZPtr cloud = input;
    PCLPointXYZPtr cloud_p (new PCLPointXYZ);
    PCLPointXYZPtr cloud_f (new PCLPointXYZ);

    //Select detection area
    //    std::cerr << "Cloud before filtering: " << std::endl;
    //    std::cerr << *cloud << std::endl;
    applyPassFilter(cloud);    
    //    std::cerr << "Cloud after filtering: " << std::endl;
    //    std::cerr << *cloud << std::endl;

    //detect planes
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

    int count=0, nr_points = (int) cloud->points.size ();

    // While 30% of the original cloud is still there
    while (cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
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

//        //Color planes
//        copyPointCloud(*cloud_p, cluster);

//        uint8_t r = rand() % 255;
//        uint8_t g = rand() % 255;
//        uint8_t b = rand() % 255;
//        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

//        for (size_t i = 0; i < cluster.points.size(); i++)
//            cluster.points[i].rgb = *reinterpret_cast<float*>(&rgb);

//        if(count==0)
//            copyPointCloud(cluster, color_cloud);
//        else
//            color_cloud += cluster;
        planes.push_back(*cloud_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        count++;
    }

    // find the largest plane
    int plane_number = planes.size();
    std::vector<int> cloud_sizes;
    std::cerr << "number of planes: "<<plane_number << std::endl;

    for(int i=0;i<plane_number;i++)
    {
        cloud_sizes.push_back(planes.at(i).points.size ());
    }

    vector<int>::const_iterator it2;
    it2 = max_element(cloud_sizes.begin(), cloud_sizes.end());
    int max_plane_index = std::distance(cloud_sizes.begin(), max_element(cloud_sizes.begin(), cloud_sizes.end()));

    cout << " the max is " << *it2 << " at position "
         << max_plane_index << std::endl;

    floor = planes.at(max_plane_index);

//    pcl::toPCLPointCloud2(*cloud,output_pcl);
//    pcl_conversions::fromPCL(output_pcl,output);
////    pcl::toROSMsg(*cloud_filtered,output);   //deprecated method to do conversion

    // Publish the data
    pub_plane.publish (floor);
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
    pass.setFilterLimits (0, 0.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_plane_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points/filtered_pcl", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_plane = nh.advertise<PCLPointXYZ> ("/camera/depth/points/floor", 1);

  // Spin
  ros::spin ();
}

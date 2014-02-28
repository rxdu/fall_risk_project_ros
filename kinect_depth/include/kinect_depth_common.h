#ifndef KINECT_DEPTH_COMMON_H
#define KINECT_DEPTH_COMMON_H

// ROS specific includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZ> PCLPointXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointXYZPtr;
typedef pcl::PointCloud <pcl::PointXYZRGB> PCLPointXYZRGB;
typedef pcl::PointCloud <pcl::PointXYZRGB>::Ptr PCLPointXYZRGBPtr;


#endif // KINECT_DEPTH_COMMON_H

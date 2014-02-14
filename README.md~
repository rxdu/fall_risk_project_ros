FallRiskProjectROS
==================

RIVeR Lab, WPI


1.Launch Files
------------------

You can find most launch files in fall_risk_assessment package.

Set up a webcam on a computer with MJPEG Server and ROSBridge running:

```
roslaunch fall_risk_assessment pc_rosbridge_mjpeg.launch 
```

Draw a circle with distance on a image from kinect:

```
roslaunch fall_risk_assessment kinect_dist_circle.launch 
```

Draw reference lines on a image from kinect:

```
roslaunch fall_risk_assessment kinect_ref_line.launch 
```


2.How to use OpenCV in a ROS node
------------------

This article can give you a idea about how it works:

http://siddhantahuja.wordpress.com/2011/07/20/working-with-ros-and-opencv-draft/

But it is out of dated and using the old rosbuild rather than catkin. 

If you still want to follow this article, make sure to make adjustments.

For example, the tutorial gives:

```
$ sudo apt-get install ros-fuerte-camera-umd
```

Then you need to change it to:

```
$ sudo apt-get install ros-groovy-camera-umd
```

Follow this tutorial to lean how to use cv_bridge

http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages


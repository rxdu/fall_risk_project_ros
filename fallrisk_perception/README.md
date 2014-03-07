FallRiskProjectROS
==================

[RIVeR Lab](http://robot.wpi.edu), WPI

**fallrisk_bringup**: mainly launch files at present

**kinect depth**: nodes about 3d depth/distance processing/vision

**kinect_image**: nodes about 2d image processing/vision

**uvc_camera**: catkinized version of tutorialROSOpenCV, mainly used for testing with webcam when robot is not available

1.Launch Files
------------------




2.Using OpenCV in a ROS node
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


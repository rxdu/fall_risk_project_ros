FallRiskProjectROS
==================

[RIVeR Lab](http://robot.wpi.edu), WPI


1.Launch Files
------------------

**Robot online mode**

Turn on the turtlebot and run basic nodes (mjpeg server, rosbridge, user nodes etc.):

```
roslaunch fallrisk_bringup startup_online.launch 
```

Just for convenience:

a. If you want to launch startup_online.launch and start amcl navigation node at the same time, launch 

```
roslaunch fallrisk_bringup startup_online_nav.launch 
```

b. If you want to launch startup_online.launch and start gmapping node at the same time, launch 

```
roslaunch fallrisk_bringup startup_online_map.launch 
```

**Robot offline (simulation) mode**

Turn on the simulator and run everything else (mjpeg server, rosbridge, user nodes etc.):

```
roslaunch fallrisk_bringup startup_offline.launch 
```


2.Source Folder
-------------------

**fallrisk_bringup**: launch files to bringup the system 

**fallrisk_common**: drivers, denpendent libraries etc.

**fallrisk_perception**: nodes about image processing, environment awareness

**fallrisk_mapping_navigation**: nodes about mapping and localization, planning and navigation

**fallrisk_tools**: tools for web interface

**fallrisk_embedded**: nodes run on embedded board

3.Installing Dependencies
-------------------

Install robot web tools:

```
sudo apt-get install ros-hydro-mjpeg-server
sudo apt-get install ros-hydro-rosbridge-server
sudo apt-get install ros-hydro-robot-pose-publisher
sudo apt-get install ros-hydro-octomap-server
```

Install turtlebot apps and simulator:

```
sudo apt-get install ros-hydro-turtlebot ros-hydro-turtlebot-apps ros-hydro-turtlebot-simulator ros-hydro-turtlebot-navigation ros-hydro-uvc-camera ros-hydro-audio-common gstreamer0.10
```

For the laptop on the turtlebot, you also need to install:

```
sudo apt-get install ros-hydro-turtlebot-viz ros-hydro-kobuki-ftdi
```

To set up kobuki base:

```
. /opt/ros/hydro/setup.bash
rosrun kobuki_ftdi create_udev_rules
```

4.Setting Up Your Workspace
-------------------

* Create your workspace

```
mkdir -p ~/fallrisk_ws/src
cd ~/fallrisk_ws/src
catkin_init_workspace
cd ~/fallrisk_ws/
catkin_make
source devel/setup.bash
```

* Setting Up .bashrc File

```
echo source ~/fallrisk_ws/devel/setup.bash >> ~/.bashrc
. ~/.bashrc
```

```
cd ~/fallrisk_ws/src
wstool init
wstool set fall_risk_project_ros https://github.com/rxdu/FallRiskProjectROS.git --git
wstool update
```

* Compile project

```
cd ~/fallrisk_ws/src/fall_risk_project_ros
git checkout hydro_devel
cd ~/fallrisk_ws
catkin_make
```

5.Using Eclipse for ROS (Hydro) projects 
-------------------

(**reference from WPI DRC project wiki**)

**For the whole workspace:**

From the ros_workspace folder run:

```
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
```

You can then import the eclipse project that was created in the build folder. The files you need to edit are under "[Source directory]". Note that all packages are under the same project.

**For just one package:**

Find the location where you wish to store your eclipse .project files. For example if you want to use this~/Workspace/eclipse/<package_name>:

```
cd ~/Workspace/eclipse
mkdir <package_name>
cd <package_name>
cmake /home/rdu/fallrisk_ws/src/fall_risk_project_ros/<package_name>/ -G "Eclipse CDT4 - Unix Makefiles"
```

Now open eclipse:

```
eclipse&
```

```
right click on 'project explorer' > import > General > Existing Projects  into Workspace > Next 
point to the directory where you created the .project file 
click Finish
```

Wait till it indexes and you should be good to go. If it does not index correctly it means the CMakeLists.txt is not written correctly.




FallRiskProjectROS
==================

[RIVeR Lab](http://robot.wpi.edu), WPI


1.Launch Files
------------------

You can find most launch files in fall_risk_assessment package.

Turn on the turtlebot and run everything else (mjpeg server, rosbridge, user nodes etc.):

```
roslaunch fallrisk_bringup startup_online.launch 
```

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

**fallrisk_embedded**: nodes run on embedded board

3.Installing Dependencies
-------------------

Install robot web tools:

```
sudo apt-get install ros-hydro-mjpeg-server
sudo apt-get install ros-hydro-rosbridge-server
sudo apt-get install ros-hydro-robot-pose-publisher
```

Install turtlebot simulator:

```
sudo apt-get install ros-hydro-turtlebot-simulator
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

THIS IS COOLL!!!!


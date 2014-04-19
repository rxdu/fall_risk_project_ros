Touchscreen Configuration
=======

This document provides step by step instructions to configure Mimo magic touch touchscreen on ubuntu 12.04. This can be used for most touch screen by DisplayLink as well. 

>Only one display can be active at a time. If we configure Touchscreen, laptop screen will be blank and vice versa.
>Take backup of /etc/X11/xorg.conf file as it would be required to revert things back to normal


Method1
------

1. Download the touchscreen folder to local machine at ~/fallrisk_ws/src/fall_risk_project_ros/touchscreen location 
2. Connect the touchscreen to laptop
3. Run the following command
    
    ```sh
    sudo cp /etc/X11/xorg.conf ~/fallrisk_ws/src/fall_risk_project_ros/hardware_configuration/touchscreen/xorg.conf_laptop 
    sudo cp ~/fallrisk_ws/src/fall_risk_project_ros/hardware_configuration/touchscreen/xorg.conf_touchScreen /etc/X11/xorg.conf
    ```
4. Restart the laptop 

To revert the settings:

1. Run the following command
    
    ```sh
    sudo cp /etc/X11/xorg.conf ~/fallrisk_ws/src/fall_risk_project_ros/hardware_configuration/touchscreen/xorg.conf_touchScreen 
    sudo cp ~/fallrisk_ws/src/fall_risk_project_ros/hardware_configuration/touchscreen/xorg.conf_laptop /etc/X11/xorg.conf
    ```
2. Restart the laptop 


Method 2
-------
1. Connect the touchscreen to laptop/computer. This should bring up green screen on the touch display.

2. Run the following command to check the location of the device

   ```
   dmesg | grep "udlfb: /dev" 
   ```
3. Note down the location of the device (/dev/fb0 or /dev/fb1)
4. Take a backup of existing /etc/X11/xorg.conf file
5. Copy the below contents and save it as /etc/X11/xorg.conf

    ```sh
    #################################################
    Section "ServerLayout"
        Identifier      "Server Layout"
        Screen  0       "DisplayLinkScreen" 0 0
        Option          "Xinerama" "on"
    EndSection

    #################################################

    Section "Files"
        ModulePath      "/usr/lib/xorg/modules"
        ModulePath      "/usr/local/lib/xorg/modules"
        ModulePath	"/usr/local/lib/xorg/modules/drivers"
    EndSection

    ################ DisplayLink ###################
    Section "Device"
        Identifier      "DisplayLinkDevice"
        Driver          "fbdev" 
        BusID           "USB"               # needed to use multiple DisplayLink devices 
        Option          "fbdev" "/dev/fb1"  # change to whatever device you want to use
    EndSection
 
    Section "Monitor"
        Identifier      "DisplayLinkMonitor"
    EndSection
 
    Section "Screen"
        Identifier      "DisplayLinkScreen"
        Device          "DisplayLinkDevice"
        Monitor         "DisplayLinkMonitor"
        DefaultDepth    16
    EndSection
    ```
6. Restart the laptop


To revert the settings:

1. replace /etc/X11/xorg.conf file with the backup file

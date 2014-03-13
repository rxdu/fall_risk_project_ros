FallRiskProjectROS
==================

[RIVeR Lab](http://robot.wpi.edu), WPI

1.Luminosity Sensor
------------------

Board: Sparkfun RedBoard, Sparkfun TSL2561 luminosity sensor board
Dependencies: 

Run node to start serial port and 

```
rosrun rosserial_python serial_node.py _port:=value _baud:=value
```

For example: 

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```

Published topic:

/sensors/luminosity

Message type:

/std_msgs/float32





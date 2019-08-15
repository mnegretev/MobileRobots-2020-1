# Mobile Robots Course
Facultad de Ingeniería UNAM, 2020-1

To use this software you need:

* Ubuntu 16.04
* ROS Kinectic
* A lot of patience

## Installation:

First, you need to install the robotino api libraries:
```bash
$ wget -qO - http://packages.openrobotino.org/keyFile | sudo apt-key add -
$ sudo su
$ echo "deb http://packages.openrobotino.org/xenial xenial main" > /etc/apt/sources.list.d/openrobotino.list
$ exit
$ sudo apt-get update
$ sudo apt-get install robotino-api2 robotino-common
$ sudo apt-get install ros-kinetic-dynamixel-sdk
```
Now, you can compile the code:
```bash
$ cd MobileRobots-2020-1/catkin_ws
$ catkin_make --pkg robotino_msgs
$ catkin_make -j2 -l2
```

## Testing
If everything is ok, run this command:
```bash
roslaunch surge_et_ambula getting_started.launch
```

and you should see something like this:

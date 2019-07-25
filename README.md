ROS2 for .NET
=============

[![CircleCI](https://circleci.com/gh/samiamlabs/ros2_dotnet/tree/dashing.svg?style=svg)](https://circleci.com/gh/samiamlabs/ros2_dotnet/tree/dashing)

TODO - update README. This works a bit for Dashing now - examples of talker and listener and message generation. (lots to do still)

Most important Issues 
- build dependencies are broken, currently needs to run colcon build twice.
- Nested types and arrays (Edit: NamedTypes and NamespacedTypes generate successfuly now)

Notice
------

This fork replaces https://github.com/esteve/ros2_dotnet/ implementation of rclcs with an improved version from https://github.com/DynoRobotics/unity_ros2, and includes some of my own fixes and changes, and a body of my original work porting the entire thing to Dashing and changing things to my preference. Since the fork contains rather revolutionary, large changes and includes a body of third person's work, it needs plenty of effort to integrate with the ros2_dotnet main.

I did this for my own purpose, but if someone is interested, here it is. I also only test the project with existing Dashing installation, in a minimal build. Please refer to original projects and their README.

Linux
-----

Make sure to source your ROS2 Dashing environment

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/adamdbrw/ros2_dotnet/dashing/ros2_dotnet_dashing.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet_dashing.repos
colcon build

```
Running. Source install/setup.bash in both terminals:

Talker:
```
ros2 run rcldotnet_examples rcldotnet_talker
```
Listener
```
ros2 run rcldotnet_examples rcldotnet_listener
```

Testing. Source install/setup.bash in terminal and change directory to rcldotnet_tests under src:

```
dotnet watch test
```
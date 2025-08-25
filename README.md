
This repo includes the core library `libgripper/` and a simple ros package `gripper_ros/` that wraps the behavior of the core library.
# ROS instructions

First build the core library and set the install location to your caktin workspace
```bash
cd libgripper
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<your-catkin-ws>/devel
make
make install
```

There are two ros packages, `gipper_ros` which includes the bhand_node, and `gripper_ros_common` which includes messages and services.
Create symbolic links in your catkin workspace src pointing to gripper_ros and gripper_ros_common.
```bash
cd <your-catkin-ws>/src
ln -s <path-to-gipper>/gripper_ros gripper_ros
ln -s <path-to-gipper>/gripper_ros_common gripper_ros_common
```
If you only need the messages and services, than you can omit the link to gipper_ros. A common use case is when the node is running on a separate machine and you need to communicate with the node on your local machine. See [gipper_ros_common readme](gripper_ros_common/README.md) for instruction to include messages in another package.

You can now build the ros node as usual
```bash
cd <your-catkin-ws>
catkin_make
```
See [gripper_ros readme](gipper_ros/README.md) for instructions on using the BHand with ROS.
# Core Library
The library exposes two classes to interact with the hand `gripper::barrett::BarrettHand` and `gripper::barrett::BarrettHandDriver`.


`gripper::barrett::BarrettHand` provides a high level interface for the hand and should be the class used in almost all cases. The BarrettHandDriver class is responsible for the lower level communication and parsing.

The Barrett Hand 262 exposes two modes, supervisory and real-time. Supervisory mode blocks until a command is complete. For example, when initializing the hand, another command cannot be sent until the entire sequence is complete. During this time you cannot query the devices for state information (such a joint position and velocity). 
The real-time mode is used to provide reference values to lower level controllers on the device. The commands can be specified to return state information in a non-blocking fashion. For that reason the BarrettHand is designed to be in real-time mode by default, continually querying the device for the current position. 
The primary way to stay in real-time mode is to use the setPosition and setVelocity methods. These calls set the reference value for PID controllers that are then passed to the device. Importantly, these calls do not generate a trajectory, so care must be taken to ensure the provided target is not too far from the current state. 
All other calls, such as open() close(), moveTo(), etc, will exit real-time mode, issue the supervisory command, and the immediately return to real-time mode. 
These supervisory calls can be convenient in that they do follow a smooth trajectory, however state information will not be updated until the entire move is complete.

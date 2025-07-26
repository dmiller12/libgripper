# Gripper_ros

## Summary

This ROS package provides a ROS interface for the BarrettHand BH8-series gripper. It exposes topics for controlling the gripper's position and velocity, publishes the gripper's state, and offers services for common actions like initialization and opening/closing the grasp.

## Dependencies

Before building, ensure you have the following dependencies in your catkin workspace:

- `gripper_ros_common`: Contains common message types for gripper control.
- `gripper`: The underlying C++ library for the BHand.

## How to Build

To build this package, copy/link it into your catkin workspace's `src` directory and build:

```bash
cd ~/catkin_ws
catkin_make --pkg gripper_ros
```

## How to Run

The recommended way to run the node is with the provided launch file. This allows for easy configuration of the serial port and other parameters.

```bash
roslaunch gripper_ros bhand.launch 
```


## `bhand_node`

This is the main bhand node that interfaces with the hardware.

### Published Topics

- `~joint_states` (`sensor_msgs/JointState`)
    Publishes the current position and velocity of all hand joints.

### Subscribed Topics

- `~position_setpoint` (`gripper_ros_common/PositionSetpoint`)
    Sets the target position for the fingers and spread. The `grasp` field controls the three fingers simultaneously.

- `~velocity_setpoint` (`gripper_ros_common/VelocitySetpoint`)
    Sets the target velocity for the fingers and spread. The `grasp` field controls the three fingers simultaneously.

### Services

- `~initialize` (`std_srvs/Empty`)
    Connects to and initializes the BarrettHand hardware. This must be called before the hand will move.

- `~open_grasp` (`std_srvs/Empty`)
    Commands the three fingers to open fully.

- `~close_grasp` (`std_srvs/Empty`)
    Commands the three fingers to close fully.

- `~open_spread` (`std_srvs/Empty`)
    Commands the finger spread to open fully.

- `~close_spread` (`std_srvs/Empty`)
    Commands the finger spread to close fully.

### Parameters

- `~port` (string, default: `/dev/ttyUSB0`)
    The serial port device name that the BarrettHand is connected to.

- `~init_prompt` (bool, default: `true`)
    If `true`, the node will print a message on startup and wait for the user to press the Enter key before attempting to initialize the hand. If `false` you must call the `~initialize` service

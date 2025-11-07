# Gripper_ros

## Summary

This ROS package provides ROS interfaces for the BarrettHand BH8-series gripper and the Robotiq 2F-140 parallel gripper. It exposes topics for controlling the grippers' position and velocity, publishes state, and offers services for common actions like initialization and opening/closing the grasp.

## Dependencies

Before building, ensure you have the following dependencies in your catkin workspace:

- `gripper_ros_common`: Contains common message types for gripper control.
- `gripper`: The underlying C++ library providing Barrett and Robotiq drivers.

## How to Build

To build this package, copy/link it into your catkin workspace's `src` directory and build:

```bash
cd ~/catkin_ws
catkin_make --pkg gripper_ros
```

## How to Run

The recommended way to run the nodes is with the provided launch files. These allow for easy configuration of the serial port and other parameters.

```bash
roslaunch gripper_ros bhand.launch 
```

To launch the Robotiq driver:

```bash
roslaunch gripper_ros robotiq.launch
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

- `~init_prompt` (bool, default: `false`)
    If `true`, the node will print a message on startup and wait for the user to press the Enter key before attempting to initialize the hand. If `false` you must call the `~initialize` service yourself.

## `robotiq_node`

Provides a ROS wrapper around the Robotiq 2F-140 gripper.

### Published Topics

- `~joint_states` (`sensor_msgs/JointState`)
    Publishes the current opening width (reported as meters) and an effort estimate based on gripper current draw.

### Subscribed Topics

- `~position_setpoint` (`gripper_ros_common/PositionSetpoint`)
    Sets the target finger separation using the `grasp` field. Interpretation of the value (normalized, millimetres, or metres) is controlled via the `~position_units` parameter.

- `~velocity_setpoint` (`gripper_ros_common/VelocitySetpoint`)
    Updates the default closing speed (`grasp`) and force (`spread`) scalars used for subsequent position commands.

### Services

- `~initialize` (`std_srvs/Empty`)
    Connects to and activates the gripper.

- `~open_grasp` (`std_srvs/Empty`)
    Opens the gripper using the configured default speed/force.

- `~close_grasp` (`std_srvs/Empty`)
    Closes the gripper using the configured default speed/force.

- `~stop` (`std_srvs/Empty`)
    Stops the current motion by clearing the go-to flag.

### Parameters

- `~port` (string, default: `/dev/ttyUSB0`)
    Serial device path for the RS485 adapter.

- `~baud_rate` (int, default: `115200`)
    Serial baud rate.

- `~slave_id` (int, default: `9`)
    Modbus slave ID configured on the gripper.

- `~default_speed` (double, default: `0.3`)
    Normalized closing speed [0–1] used for position commands.

- `~default_force` (double, default: `0.5`)
    Normalized force [0–1] used for position commands.

- `~auto_activate` (bool, default: `true`)
    Automatically activate the gripper after connecting.

- `~activation_speed` (double, default: `0.3`)
    Speed command used during activation.

- `~activation_force` (double, default: `0.5`)
    Force command used during activation.

- `~position_units` (string, default: `normalized`)
    Unit for the `grasp` field in `position_setpoint` messages. Valid values: `normalized` (0–1), `mm`, or `m`.

- `~publish_rate` (double, default: `25.0`)
    Joint-state publish rate in Hz.

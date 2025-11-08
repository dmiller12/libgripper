# Gripper_ros

## Summary

This ROS package provides a ROS interface for the BarrettHand BH8-series gripper and the Robotiq 2F-140 gripper. It exposes topics for controlling the grippers' position and velocity, publishes joint states, and offers services for common actions like initialization and opening/closing the grasp.

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

To run the Robotiq node (Modbus RTU over serial):

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

ROS interface for the Robotiq 2F-140 gripper. The node drives the gripper over a serial Modbus RTU connection using the new `gripper::robotiq::RobotiqGripper` backend.

### Published Topics

- `~joint_states` (`sensor_msgs/JointState`)
    Publishes two virtual finger joints representing the current finger spacing (mirrored about the palm center). Effort is populated with the gripper's supply current estimate.

### Subscribed Topics

- `~position_setpoint` (`gripper_ros_common/PositionSetpoint`)
    The `grasp` field represents the desired jaw width in meters (`0.0` closed, `0.14` fully open). The `spread` field is ignored.

- `~velocity_setpoint` (`gripper_ros_common/VelocitySetpoint`)
    The signed command is integrated directly into jaw-width targets so velocity-only joystick pipelines (e.g., Barrett teleop) can move the gripper without modification. By default the `grasp` channel is used (positive closes, negative opens) but the node can optionally consume `spread` instead.

### Services

- `~initialize` (`std_srvs/Empty`)
    Opens the serial port, pushes the configured speed/force ratios, and activates the Robotiq gripper.

- `~open_grasp` / `~close_grasp` (`std_srvs/Empty`)
    Convenience services for fully opening/closing the gripper.

### Parameters

- `~port` (string, default: `/dev/ttyUSB0`)
    Serial device used for Modbus RTU.

- `~baud_rate` (int, default: `115200`)
    Serial baud rate.

- `~default_speed_ratio` (double, default: `0.4`)
    Normalized speed (0–1) that is pushed during initialization.

- `~default_force_ratio` (double, default: `0.5`)
    Normalized force (0–1) used for every command.

- `~velocity_gain` (double, default: `0.04`)
    Linear speed (meters/second) that corresponds to a unit-magnitude velocity command. Increase to make joystick control snappier.

- `~velocity_use_spread` (bool, default: `false`)
    Use the `spread` field of `VelocitySetpoint` instead of `grasp` when integrating velocity commands.

- `~invert_velocity_direction` (bool, default: `false`)
    Flip the sign of incoming velocity commands (useful if your controller axis feels backwards).

- `~min_width` / `~max_width` (double, defaults: `0.0` and gripper stroke)
    Clamp bounds for the integrated jaw width.

- `~initial_width` (double, default: `~max_width`)
    Starting width used before any state feedback is received.

- `~invert_service_direction` (bool, default: `false`)
    When `true`, swaps the behavior of `~open_grasp` and `~close_grasp` to accommodate installations where the wiring feels reversed.

- `~auto_initialize` (bool, default: `false`)
    Automatically calls the initialize service on startup when true.

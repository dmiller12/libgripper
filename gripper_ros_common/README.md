Provides messages and services for the barrett hand.

# Instructions

Add the package to <your-catkin-ws>/src

To use these messages, add the following to your package:
## Add to package.xml

```xml
<build_depend>gripper_ros_common</build_depend>
<run_depend>gripper_ros_common</run_depend>
```

## Add to CMakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  gripper_ros_common
)
```


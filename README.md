
This includes the core library `libgripper/` and a simple ros package `gripper_ros/` that wraps the behavior of the core library 
# ROS instructions

First build the core library and set the install location to your caktin workspace
```bash
cd libgripper
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<your-catkin-ws>/devel
make
make install
```

Then create a symbolic link in your workspace src pointing to gripper_ros
```bash
cd <your-catkin-ws>/src
ln -s <path-to-gipper>/gripper_ros gripper_ros
```

You can now build the ros node as usual
```bash
cd <your-catkin-ws>
catkin_make
```

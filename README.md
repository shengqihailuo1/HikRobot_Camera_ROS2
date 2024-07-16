# HikRobot_Camera_ROS2

## install

```_
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/shengqihailuo1/HikRobot_Camera_ROS2.git
cd HikRobot_Camera_ROS2
colcon build
source install/setup.bash
```

## launch

```
# NOTE：Before starting the node, you need to look up the Hikon camera serial number and replace "expect_serial_number" in my_camera.yaml

ros2 run HikRobot_Camera_ROS2 hk_camera --ros-args --params-file HikRobot_Camera_ROS2/config/my_camera.yaml
# or
ros2 launch HikRobot_Camera_ROS2 my_camera_launch.py
```


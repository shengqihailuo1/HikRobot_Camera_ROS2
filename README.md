# HikRobot_Camera_ROS2

## install

```_
mkdir -p ~/your_ros2_ws/src
cd ~/your_ros2_ws/src
git clone https://github.com/shengqihailuo1/HikRobot_Camera_ROS2.git
cd HikRobot_Camera_ROS2
colcon build
source install/setup.bash
```

## launch

```
ros2 run HikRobot_Camera_ROS2 hk_camera --ros-args --params-file HikRobot_Camera_ROS2/config/my_camera.yaml
# or
ros2 launch HikRobot_Camera_ROS2 my_camera_launch.py
```


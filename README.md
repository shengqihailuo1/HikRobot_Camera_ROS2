# HikRobot_Camera_ROS2

## install

```_
mkdir -p ~/your_ros2_ws/src
cd ~/your_ros2_ws/src
git clone https://github.com/shengqihailuo1/HikRobot_Camera_ROS2.git
cd ~/your_ros2_ws
colcon build
source install/setup.bash
```

## launch

```
ros2 run my_hk_ros2 hk_camera --ros-args --params-file HikRobot_Camera_ROS2/config/my_camera.yaml
# or
ros2 launch my_hk_ros2 my_camera_launch.py
```


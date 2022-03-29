# sfa3d_ros

ros wrapper for [Super Fast and Accurate 3D Object Detection based on 3D LiDAR Point Clouds(SFA3D)](https://github.com/maudzung/SFA3D).

## Environment

- Ubuntu 18.04 + Melodic
- Ubuntu 20.04 + Noetic

### Workspace build

#### Workspace build (Melodic)

```bash
sudo apt-get install python3-catkin-pkg-modules python3-rospkg-modules python3-venv python3-empy
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-catkin
source /opt/ros/melodic/setup.bash
mkdir -p ~/sfa3d_ws/src
cd ~/sfa3d_ws/src
wstool init
wstool set sfa3d_ros https://github.com/tongtybj/sfa3d_ros.git --git
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/sfa3d_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.7m.so
catkin build
```

#### Workspace build (Noetic)

```bash
sudo apt-get install ros-noetic-catkin
source /opt/ros/noetic/setup.bash
mkdir -p ~/sfa3d_ws/src
cd ~/sfa3d_ws/src
wstool init
wstool set sfa3d_ros https://github.com/tongtybj/sfa3d_ros.git --git
wstool update
rosdep install --from-paths . --ignore-src -y -r
cd ~/sfa3d_ws
catkin init
catkin build
```

### Usage

```bash
roslaunch sfa3d_ros 3d_lidar_object_detection.launch
```

#### argument:

`INPUT_TOPIC`: topic name of the 3D Lidar (e.g., `/velodyne_points`)

#### sample:

please download rosbag file of [rosbag play 2019-03-01-velodyne_imu_usb_cam_eng8-2-outdoor.bag](https://drive.google.com/file/d/16fqCgZhWdCg1KW3h_FZwKsADMUw8H060/view?usp=sharing)

```bash
roslaunch sfa3d_ros 3d_lidar_object_detection.launch INPUT_TOPIC:=/velodyne_points
rosbag play 2019-03-01-velodyne_imu_usb_cam_eng8-2-outdoor.bag
```





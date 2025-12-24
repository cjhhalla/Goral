# Goral Navigation & SLAM Setup (ROS 2 Humble)

This repository provides a local (non-Docker) setup guide for running
navigation and SLAM using **ROS 2 Humble**, **Gazebo**, and **mesh-based navigation**.

The original environment was defined in Docker.  
This document explains how to install and run everything **directly on the host system**.

---

## 🧩 Environment

- OS: Ubuntu 22.04
- ROS 2: Humble Hawksbill
- Simulator: Gazebo (ROS 2 integration)
- SLAM / Navigation: mesh_navigation, SLAMesh (docker)
---

## 1️⃣ Install ROS 2 Dependencies

Make sure ROS 2 Humble is already installed and sourced.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-localization \
  ros-humble-ros2-controllers \
  ros-humble-ros2-control \
  ros-humble-velodyne \
  ros-humble-velodyne-gazebo-plugins \
  ros-humble-velodyne-description

pip3 install tf-transformations
```

## 2️⃣ Workspace Setup

Clone this repository
```bash
git clone --recursive https://github.com/cjhhalla/Goral.git
```

## 3️⃣ Import Source Dependencies
Inside the <mesh_navigation> directory:
```bash
cd mesh_navigation
vcs import --input source_dependencies.yaml
```

## 4️⃣ Install ROS Dependencies via rosdep
From the workspace root:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 5️⃣ Build the Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```
## 6️⃣ Run Navigation & SLAM
```bash
ros2 launch go2_config gz_lidar_odom.launch.py

ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py world_name:=tray
```

## 7️⃣ Change Mesh Map (with docker)
If you want to use the prebuilt SLAMesh (ROS1) environment:
```bash
pip3 install rosbags
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

docker pull junhyeokchoe/slamesh:latest

cd slam
./run.sh

# in run.sh
# you should change dataset dir ex) --volume=/home/jun/ros1_convert:/root/dataset/ \

cd /root/Workspaces/slamesh_ws/src/SLAMesh/launch

source /opt/ros/melodic/setup.bash
source ~/Workspaces/slamesh_ws/devel/setup.bash

# in slamesh_online.launch change underline respect to your ROS2 Bag
# <remap from="/velodyne_points" to="/os_cloud_node/points" />

roslaunch slamesh_online.launch

# docker exec -it slamesh /bin/bash
rosbag play <your_rosbag>

# After building map in your local dir connected with docker
cp 00_mesh.ply <your mesh_navigation_tutorials_pkg/maps

# ex cp 00_mesh.ply /home/jun/jun_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials/maps

# mesh_navigation wants world_name only in tray, floor_is_lava, parking_garage so

cp 00_mesh.ply tray.ply

# caution please remove .h5 file in mesh_navigation_tutorials launch dir

ros2 launch mesh_navigation_tutorials mesh_navigation_tutorials_launch.py world_name:=tray

```

### How to convert ROS2 bag -> ROS1 bag
```bash
rosbags-convert \
  --src rosbag2_2025_11_26-22_10_56/ \
  --dst ros1_output.bag \
  --src-typestore ros2_humble \
  --dst-typestore ros1_noetic
```

## 8️⃣ Todo
- Check Slamesh performance
- Check compatibility with Slamesh & mesh_navigation
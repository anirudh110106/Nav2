# System / ROS2 Dependencies

These are not pip packages — install via apt or rosdep. Use `requirements.txt` only for the Python packages layered on top.

## Core

| Dependency | Notes |
|---|---|
| Ubuntu 24.04 (Noble) | Required base OS for ROS2 Jazzy |
| ROS2 Jazzy Jalisco | `sudo apt install ros-jazzy-desktop` |
| colcon | Build tool — `sudo apt install python3-colcon-common-extensions` |
| rosdep | `sudo apt install python3-rosdep` |

## Navigation & Mapping

| Package | apt package |
|---|---|
| Nav2 | `ros-jazzy-navigation2` |
| Nav2 Bringup | `ros-jazzy-nav2-bringup` |
| SLAM Toolbox | `ros-jazzy-slam-toolbox` |
| AMCL | included in `navigation2` |
| DWB local planner | `ros-jazzy-dwb-core` (included with navigation2) |

## Sensors

| Package | apt package | Used for |
|---|---|---|
| RealSense ROS2 wrapper | `ros-jazzy-realsense2-camera` | Depth camera mode |
| depthimage_to_laserscan | `ros-jazzy-depthimage-to-laserscan` | Converts depth image → virtual LaserScan |
| rplidar_ros | bundled in this repo (`rplidar_ros/`) | LiDAR mode |

## Install (quick reference)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-realsense2-camera \
  ros-jazzy-depthimage-to-laserscan \
  python3-colcon-common-extensions \
  python3-rosdep

# Resolve any remaining deps declared in package.xml
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Python (pip)

See `requirements.txt`. Install after sourcing your ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash
pip install -r requirements.txt
```

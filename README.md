# Nav2 Integration for Lekiwi-Rover
The autonomous navigation pipeline is powered by the ROS2 Nav2 framework and SLAM (Simultaneous Localization and Mapping). This is implemented by two distinct sensor , an Intel RealSense depth camera to convert the depth to a 2d LaserScan and a dedicated LiDAR sensor. SLAM is first utilized to scan and construct a static map of the operating workspace. Once the map is established, the Nav2 stack takes over for autonomous navigation. It uses AMCL to accurately localize the robot by fusing the active sensor's scan data with continuous wheel odometry provided by the ST3215 servos. You can checkout the workflow image below .


# LeKiwi Rover — Nav2 Navigation Stack

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/index.html)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation2-orange.svg)](https://navigation.ros.org/)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-slam__toolbox-green.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![Platform](https://img.shields.io/badge/Platform-LeKiwi%20Rover-lightgrey.svg)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](#license)

SLAM and autonomous navigation stack for the **LeKiwi** omnidirectional (three-wheel holonomic) rover, built on **ROS2** and **Nav2**. Supports two sensing modes — an Intel RealSense depth camera (converted to a virtual LaserScan) and an RPLIDAR — for both mapping and navigation.

---

## Overview

This package lets the LeKiwi rover:
- Build a map of its environment using **SLAM Toolbox**, with either a depth camera or a LiDAR as the scan source.
- Navigate autonomously within a built map using the **Nav2** stack, with AMCL for localization.

It bundles a custom `nav2` ROS2 package (launch files, params, odometry) alongside the `rplidar_ros` driver package for LiDAR support.

---

## Prerequisites

- ROS2 (Jazzy or compatible)
- [Nav2](https://navigation.ros.org/) (`navigation2`, `nav2_bringup`)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- Intel RealSense ROS2 wrapper (`realsense2_camera`) — for depth camera mode
- `rplidar_ros` (included in this repo) — for LiDAR mode
- `colcon` build tools

---

## Installation

```
mkdir Nav2/src
cd /Nav2/src
git clone https://github.com/anirudh110106/Nav2

# Build and source your ros2 workspace
cd .. 
colcon build
source install/setup.bash
```

---

# Nav2 integration :-
<img width="3093" height="1036" alt="Nav2" src="https://github.com/user-attachments/assets/9b86305c-a87b-4885-be53-57cc45861de9" />

## Usage

This repository is designed for autonomous navigation on the **Lekiwi Rover** using either an **RPLiDAR** or a **Depth Camera**. It includes the required launch files and configurations for both sensors, making it easy to switch between them based on your hardware setup. The commands for running SLAM mapping and autonomous navigation with each method are provided below.

### 1. Mapping (SLAM Toolbox)

Build a map of the environment before running navigation.

**Using Depth Camera:**
```bash
ros2 launch nav2 depth_laser.py
```

**Using LiDAR:**
```bash
ros2 launch nav2 slam.py
```

Save the resulting map into the `nav2/map/` directory once mapping is complete.

## 2. Navigation (Nav2)
#### Navigation pipeline :-

<img width="3595" height="1665" alt="image" src="https://github.com/user-attachments/assets/418df01c-a1b1-4f95-8f8a-9b17f8872775" />


Run autonomous navigation on a previously built map.

**Using Depth Camera:**
```bash
ros2 launch nav2 nav2.py
```

**Using LiDAR:**
```bash
ros2 launch nav2 lidar_nav2.py
```

---

## Package Structure

```
nav2/
├── nav2/
│   ├── odom.py              # Odometry handling
│   ├── launch/
│   │   ├── depth_laser.py   # SLAM launch — depth camera → virtual LaserScan
│   │   ├── slam.py          # SLAM launch — LiDAR
│   │   ├── nav2.py          # Navigation launch — depth camera
│   │   └── lidar_nav2.py    # Navigation launch — LiDAR
│   ├── map/                 # Saved maps (.pgm + .yaml)
│   └── params/
│       ├── dwb_params.yaml      # DWB local planner params
│       └── my_nav2_params.yaml  # Nav2 stack params (AMCL, planner, controller, etc.)
├── resource/
├── test/                    # Standard ROS2 lint/copyright tests
├── package.xml
├── setup.py
└── setup.cfg

rplidar_ros/                 # RPLIDAR driver package (SDK, launch files, udev rules)
```

---

## Maps

Pre-built maps are stored in `nav2/nav2/map/` :
These maps can be used as a benchmark or visual guidance to compare with your own recorded map . These maps will not work in your environment and can effect the behaviour of the rover due to the conflict of local and global maps .
<img width="445" height="386" alt="map" src="https://github.com/user-attachments/assets/58cd9f52-0acb-4f6a-994d-03b5a3cae643" />


- `lidar_map.pgm` / `lidar_map.yaml` — map built with LiDAR
- `my_new_map.pgm` / `my_new_map.yaml` — additional/updated map

---

## Notes

- Depth camera mode converts RealSense depth data into a virtual `LaserScan` before feeding it into SLAM/Nav2, so the same navigation stack works with either sensor.
- Localization uses AMCL against a pre-built map; make sure the correct map file is referenced in your Nav2 params before launching navigation.
- Tune `dwb_params.yaml` for local trajectory behavior and `my_nav2_params.yaml` for global stack configuration (costmaps, planner, recovery behaviors).

---

## License
This project is intended for educational and robotics research purposes. However the rplidar package is used for the liDAR integreation.
MIT — see [LICENSE](rplidar_ros/LICENSE) for the bundled `rplidar_ros` driver's license terms.

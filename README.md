# LiDAR-Based Lane Navigation

This repository contains a ROS-based autonomous vehicle navigation system that uses LiDAR point clouds for lane detection and vehicle control on Polaris GEM e2 and e4.

## Demo

[![System Demo](https://img.youtube.com/vi/cCTi2zFftlY/0.jpg)](https://www.youtube.com/watch?v=cCTi2zFftlY)

Click the image above to watch our system demonstration video.

## System Overview

This system integrates multiple components for autonomous navigation:

![Pipeline Diagram](https://github.com/user-attachments/assets/2270f090-0446-4392-a38a-c3dde9e49b30)

Key components:
- Pre-processing and filtering of LiDAR point clouds
- Point Transformer V3 for inference
- KISS-ICP for online SLAM
- Frame matching with buffered mapping
- Lane detection and waypoint generation
- Vehicle control

## Prerequisites

- ROS (tested on ROS Noetic)
- Python 3.8+
- Conda package manager
- NVIDIA GPU (recommended)

## Installation

1. Clone the repository and set up the ROS workspace:
```bash
cd demo_ws
source devel/setup.bash
```

2. Create and activate the Pointcept Conda environment:
```bash
conda env create -f pointcept151.yml -n pointcept151
conda activate pointcept151
```

## Usage

### Initial Setup
1. Drive the vehicle to an open area away from buildings
2. Remain stationary for 30 seconds to allow sensor calibration

### Launch Sequence

1. Initialize sensors:
```bash
source devel/setup.bash
roslaunch basic_launch sensor_init.launch
```

2. Launch visualization tools:
```bash
source devel/setup.bash
roslaunch basic_launch visualization.launch
```

3. Enable joystick control:
```bash
source devel/setup.bash
roslaunch basic_launch dbw_joystick.launch
```

### Navigation Pipeline

1. Launch point cloud preprocessing:
```bash
conda activate pointcept151
python3 src/pointcept151/inference_ros_filter.py
```

2. Start sequence matching:
```bash
conda activate pointcept151
python3 src/sequence_matching.py
```

3. Launch real-time window search:
```bash
conda activate pointcept151
python3 src/windowSearch_realtime.py
```

4. Start KISS-ICP SLAM:
```bash
roslaunch src/kiss-icp/ros/launch/odometry.launch topic:=/ouster/points
```

5. Launch inference with near-IR model or signal by defalt (optional):
```bash
python3 src/pointcept151/inference_ros_filter.py model_type:=near_ir 
```

## System Components

### 1. LiDAR Point Cloud Processing (src/pointcept151/inference_ros_filter.py)
- Raw point cloud data collection and preprocessing by range, height, and x y

### 2. Point Transformer V3 Infernce (src/pointcept151/inference_ros_filter.py)
- State-of-the-art deep learning architecture
- Specialized for point cloud processing
- Feature extraction and scene understanding
- around 300 - 400 ms infernce time on RTX A4000
- publish infernce result

### 3. KISS-ICP SLAM (src/kiss-icp)
- Real-time simultaneous localization and mapping
- Online odometry estimation for precise positioning
- Efficient point cloud registration
- publish odemetry 

### 4. Frame Matching (src/sequence_matching.py)
- map between ouster lidar frame seqquence with the kiss icp odemetry

### 5. Lane Detection (src/windowSearch_realtime.py)
- maintained a rolling buffer of the lidar frames (mapping)
- perfrorm DBSCAN and window search for lane line detection
- pubish waypoints

### 6. Vehicle Control (src/gem_lidar_tracker_pp_new.py)
- Waypoint-based trajectory planning
- Adaptive steering and speed control (PID)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Point Transformer V3 implementation team
- KISS-ICP SLAM system developers
- ROS community and contributors

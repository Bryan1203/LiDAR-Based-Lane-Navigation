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

3. Install Point Transformer dependencies:
```bash
# Install pointops for PTv1 & PTv2 or precise evaluation
cd src/pointcept151/libs/pointops
python setup.py install

# Install Google's sparsehash and pointgroup_ops
conda install -c bioconda google-sparsehash 
cd src/pointcept151/libs/pointgroup_ops
python setup.py install --include_dirs=${CONDA_PREFIX}/include
cd ../..
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

5. Launch inference with near-IR model or signal by default (optional):
```bash
python3 src/pointcept151/inference_ros_filter.py model_type:=near_ir 
```

## System Components

### 1. LiDAR Point Cloud Processing (src/pointcept151/inference_ros_filter.py)
- Raw point cloud data collection and preprocessing by range, height, and x y

### 2. Point Transformer V3 Inference (src/pointcept151/inference_ros_filter.py)
- State-of-the-art deep learning architecture
- Specialized for point cloud processing
- Feature extraction and scene understanding
- Around 300 - 400 ms inference time on RTX A4000
- Publishes inference result

### 3. KISS-ICP SLAM (src/kiss-icp)
- Real-time simultaneous localization and mapping
- Online odometry estimation for precise positioning
- Efficient point cloud registration
- Publishes odometry 

### 4. Frame Matching (src/sequence_matching.py)
- Maps between Ouster LiDAR frame sequence with the KISS-ICP odometry

### 5. Lane Detection (src/windowSearch_realtime.py)
- Maintains a rolling buffer of the LiDAR frames (mapping)
- Performs DBSCAN and window search for lane line detection
- Publishes waypoints

### 6. Vehicle Control (src/gem_lidar_tracker_pp_new.py)
- Waypoint-based trajectory planning
- Adaptive steering and speed control (PID)

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Point Transformer V3 implementation team
- KISS-ICP SLAM system developers
- ROS community and contributors

# direct_visual_lidar_calibration

This package provides LiDAR-Camera calibration tools.

## Installation

```bash
# Install visualizer
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake ..
make -j$(nproc)
sudo make install
```

```bash
# ROS1
cd ~/catkin_ws/src
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd .. && catkin_make
```

```bash
# ROS2
cd ~/ros2_ws/src
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd .. && colcon build
```

## Usage

```bash
ros2 run direct_visual_lidar_calibration preprocess ouster_pinhole ouster_pinhole_preprocessed -a -d -v
```


```bash
ros2 run direct_visual_lidar_calibration initial_guess_manual ouster_pinhole_preprocessed
```

```bash
ros2 run direct_visual_lidar_calibration initial_guess_auto ouster_pinhole_preprocessed
```

```bash
ros2 run direct_visual_lidar_calibration calibrate ouster_pinhole_preprocessed
```

```bash
ros2 run direct_visual_lidar_calibration viewer ouster_pinhole_preprocessed
```

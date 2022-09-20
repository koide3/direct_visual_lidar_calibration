# direct_visual_lidar_calibration

## Installation

```bash
# Install visualizer
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake ..
make -j
sudo make install

# Install ROS packages
cd catkin_ws/src
git clone https://github.com/koide/glim.git --recursive
git clone https://github.com/koide3/direct_visual_lidar_calibration.git --recursive
cd ..
catkin_make
```


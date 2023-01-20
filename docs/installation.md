# Installation

## Dependencies

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) [optional]

## Install Common dependencies

```bash
# Install dependencies
sudo apt install libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt install libgtsam-dev libgtsam-unstable-dev

# Install Ceres
git clone https://github.com/ceres-solver/ceres-solver
mkdir ceres-solver/build && cd ceres-solver/build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
make -j$(nproc)
sudo make install

# Install Iridescence for visualization
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

## Install SuperGlue (Optional)

!!!warning
    [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork.git) is not allowed to be used for commercial purposes. You must carefully check and follow its licensing conditions!!

```bash
pip3 install numpy opencv-python torch matplotlib
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git

echo 'export PYTHONPATH=$PYTHONPATH:/path/to/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc
```

## Build the calibration toolbox

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
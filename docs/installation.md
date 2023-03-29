# Installation

!!!note
    We provide docker images so that the user can do calibration without installation: [Docker images](docker.md)

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
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2a9
mkdir build && cd build
# For Ubuntu 22.04, add -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install

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
    [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork.git) is not allowed to be used for commercial purposes. You must carefully check and follow its licensing conditions.

```bash
pip3 install numpy opencv-python torch matplotlib
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git

echo 'export PYTHONPATH=$PYTHONPATH:/path/to/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc
```

## Build direct_visual_lidar_calibration

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
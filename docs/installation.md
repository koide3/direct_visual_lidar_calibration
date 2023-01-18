# Installation

## Common dependencies

```bash
# Install dependencies
sudo apt install libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# Install GTSAM
git clone https://github.com/borglab/gtsam
mkdir gtsam/build && cd gtsam/build
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

## SuperGlue (Optional)

Note: [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork.git) is not allowed to be used for commercial purposes. You must carefully check and follow its licensing conditions!!

```bash
pip3 install numpy opencv-python torch matplotlib

git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git

echo 'export PYTHONPATH=$PYTHONPATH:/path/to/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc
```

## Build ROS package

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
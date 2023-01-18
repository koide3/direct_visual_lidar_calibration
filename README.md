# direct_visual_lidar_calibration

Detailed documentation will be available by the end of March 2023.

This package provides a toolbox for LiDAR-camera calibration that is: 

- **Generaliable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Automatic**: The calibration process is automatic and does not require an initial guess.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

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

# Usage

[Detailed usage]()

## Preprocessing

```bash
# -a = Automatic topic detection
# -d = Dynamic points integration
# -v = Visualization
ros2 run direct_visual_lidar_calibration preprocess -a -d -v ouster_pinhole ouster_pinhole_preprocessed
```

## Initial guess

### Manual estimation

```bash
ros2 run direct_visual_lidar_calibration initial_guess_manual ouster_pinhole_preprocessed
```

### Automatic estimation

Note: SuperGlue is not allowed to be used for commercial purposes!!

```bash
ros2 run direct_visual_lidar_calibration find_matches_superglue.py ouster_pinhole_preprocessed
ros2 run direct_visual_lidar_calibration initial_guess_auto ouster_pinhole_preprocessed
```

## Calibration

```bash
ros2 run direct_visual_lidar_calibration calibrate ouster_pinhole_preprocessed
```

## Result inspection

```bash
ros2 run direct_visual_lidar_calibration viewer ouster_pinhole_preprocessed
```

# License

This package is released under the MIT license.

# Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023

# Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan
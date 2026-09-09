# direct_visual_lidar_calibration

本 fork 包含 Fast-Dual360 的双 MEI 鱼眼外参求精扩展：从 ROS1/ROS2 bag 的开头固定静止段准备点云，基于已知内参和可靠外参初值分别求精 LiDAR → cam0/cam1，并提供结果查看器。

[双 MEI 功能与用法](docs/dual_mei_static_zh.md) · [YAML 配置说明](docs/dual_mei_configuration_zh.md) · [编译与使用指南](docs/build_and_usage_zh.md)

以下保留上游工具箱介绍及原始工作引用。

This package provides a toolbox for LiDAR-camera calibration that is: 

- **Generalizable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Automatic**: The calibration process is automatic and does not require an initial guess.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

**Documentation: [https://koide3.github.io/direct_visual_lidar_calibration/](https://koide3.github.io/direct_visual_lidar_calibration/)**  
**Docker hub: [koide3/direct_visual_lidar_calibration](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)**

[![Build](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml/badge.svg)](https://github.com/koide3/direct_visual_lidar_calibration/actions/workflows/push.yaml) [![Docker Image Size (latest by date)](https://img.shields.io/docker/image-size/koide3/direct_visual_lidar_calibration)](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)

![213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6](https://user-images.githubusercontent.com/31344317/213427328-ddf72a71-9aeb-42e8-86a5-9c2ae19890e3.jpg)

[Video](https://www.youtube.com/watch?v=7TM7wGthinc&feature=youtu.be)

## Dependencies

- [ROS1/ROS2](https://www.ros.org/)
- [PCL](https://pointclouds.org/)
- [OpenCV](https://opencv.org/)
- [GTSAM](https://gtsam.org/)
- [Ceres](http://ceres-solver.org/)
- [Iridescence](https://github.com/koide3/iridescence)
- [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) [optional]

## Getting started

1. [Installation](https://koide3.github.io/direct_visual_lidar_calibration/installation/) / [Docker images](https://koide3.github.io/direct_visual_lidar_calibration/docker/)
2. [Data collection](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
3. [Calibration example](https://koide3.github.io/direct_visual_lidar_calibration/example/)
4. [Program details](https://koide3.github.io/direct_visual_lidar_calibration/programs/)

## License

This package is released under the MIT license.

## Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023, [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

## Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan

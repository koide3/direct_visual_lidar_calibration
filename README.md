# direct_visual_lidar_calibration

Detailed documentation will be available by the end of March 2023.

This package provides a toolbox for LiDAR-camera calibration that is: 

- **Generaliable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Automatic**: The calibration process is automatic and does not require an initial guess.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

**[Documentation: https://koide3.github.io/direct_visual_lidar_calibration/](https://koide3.github.io/direct_visual_lidar_calibration/)**

![Screenshot_20230119_173721](https://user-images.githubusercontent.com/31344317/213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6.png)

# License

This package is released under the MIT license.

# Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023

# Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan
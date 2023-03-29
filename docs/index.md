# Home

This package provides a toolbox for LiDAR-Camera calibration that is: 

- **Generalizable**: It can handle various LiDAR and camera projection models including spinning and non-repetitive scan LiDARs, and pinhole, fisheye, and omnidirectional projection cameras.
- **Target-less**: It does not require a calibration target but uses the environment structure and texture for calibration.
- **Single-shot**: At a minimum, only one pairing of a LiDAR point cloud and a camera image is required for calibration. Optionally, multiple LiDAR-camera data pairs can be used for improving the accuracy.
- **Automatic**: The calibration process is automatic and does not require an initial guess.
- **Accurate and robust**: It employs a pixel-level direct LiDAR-camera registration algorithm that is more robust and accurate compared to edge-based indirect LiDAR-camera registration.

![213393920-501f754f-c19f-4bab-af82-76a70d2ec6c6](https://user-images.githubusercontent.com/31344317/213427328-ddf72a71-9aeb-42e8-86a5-9c2ae19890e3.jpg)
<span style="color: Red;"><i class="fa fa-youtube-play" aria-hidden="true"></i></span> [Video](https://youtu.be/7TM7wGthinc)


## Getting started

1. [Installation](installation.md) / [Docker images](docker.md)
2. [Data collection](collection.md)
3. [Calibration example](example.md)
4. [Program details](programs.md)

## License

This package is released under the MIT license.

## Publication

Koide et al., General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox, ICRA2023, [[PDF]](https://staff.aist.go.jp/k.koide/assets/pdf/icra2023.pdf)

## Contact

Kenji Koide, National Institute of Advanced Industrial Science and Technology (AIST), Japan
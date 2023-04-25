# Calibration example

## Dataset

[LiDAR-camera calibration dataset (Zenodo)](https://zenodo.org/record/7780490)

- [livox.tar.gz (ROS2)](https://zenodo.org/record/7779880/files/livox.tar.gz?download=1)
- [ouster.tar.gz (ROS2)](https://zenodo.org/record/7779880/files/ouster.tar.gz?download=1)
- [livox_ros1.tar.gz (ROS1)](https://zenodo.org/record/7779880/files/livox_ros1.tar.gz?download=1)
- [ouster_ros1.tar.gz (ROS1)](https://zenodo.org/record/7779880/files/ouster_ros1.tar.gz?download=1)

!!!note
    CameraInfo messages in the example ROS1 bag files exhibit an MD5 checksum that is different from that of the standard sensor_msgs/CameraInfo msg due to ROS2-ROS1 bag conversion issues. Because this affects the automatic camera info extraction, you need to manually specify camera parameters for preprocessing.

## Livox-camera calibration

### 1. Download dataset

Download [livox.tar.gz](https://zenodo.org/record/7777354/files/livox.tar.gz?download=1) from [zenodo repository](https://zenodo.org/record/7780490) and unzip it:
```bash
$ tar xzvf livox.tar.gz
```


The livox dataset contains five ros2 bags, and each bag contains points/image/camera_info topics:
```bash
$ ls livox
# rosbag2_2023_03_09-13_42_46  rosbag2_2023_03_09-13_44_10  rosbag2_2023_03_09-13_44_54  rosbag2_2023_03_09-13_46_10  rosbag2_2023_03_09-13_46_54

$ ros2 bag info livox/rosbag2_2023_03_09-13_42_46/
# Files:             rosbag2_2023_03_09-13_42_46_0.db3
# Bag size:          582.9 MiB
# Storage id:        sqlite3
# Duration:          15.650s
# Start:             Mar  9 2023 13:42:46.665 (1678336966.665)
# End:               Mar  9 2023 13:43:02.316 (1678336982.316)
# Messages:          2972
# Topic information: Topic: /livox/points | Type: sensor_msgs/msg/PointCloud2 | Count: 157 | Serialization Format: cdr
#                    Topic: /livox/imu | Type: sensor_msgs/msg/Imu | Count: 2597 | Serialization Format: cdr
#                    Topic: /livox/lidar | Type: livox_interfaces/msg/CustomMsg | Count: 157 | Serialization Format: cdr
#                    Topic: /image | Type: sensor_msgs/msg/Image | Count: 30 | Serialization Format: cdr
#                    Topic: /camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 31 | Serialization Format: cdr
```

### 2. Preprocessing

```bash
# -a : Detect points/image/camera_info topics automatically
# -v : Enable visualization
$ ros2 run direct_visual_lidar_calibration preprocess livox livox_preprocessed -av
```

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/3zCImLphvVM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>


After running ```preprocess```, you can find a directory named ```livox_preprocessed``` that containts generated dense point clouds, camera images, and some meta data ([screenshot](https://user-images.githubusercontent.com/31344317/228187524-6f02158e-b2d6-4a02-9e6f-b902539d8e13.png)):

```bash
$ ls livox_preprocessed/
# calib.json                                         rosbag2_2023_03_09-13_44_10_lidar_intensities.png  rosbag2_2023_03_09-13_44_54.png                    rosbag2_2023_03_09-13_46_54_lidar_intensities.png
# rosbag2_2023_03_09-13_42_46_lidar_indices.png      rosbag2_2023_03_09-13_44_10.ply                    rosbag2_2023_03_09-13_46_10_lidar_indices.png      rosbag2_2023_03_09-13_46_54.ply
# rosbag2_2023_03_09-13_42_46_lidar_intensities.png  rosbag2_2023_03_09-13_44_10.png                    rosbag2_2023_03_09-13_46_10_lidar_intensities.png  rosbag2_2023_03_09-13_46_54.png
# rosbag2_2023_03_09-13_42_46.ply                    rosbag2_2023_03_09-13_44_54_lidar_indices.png      rosbag2_2023_03_09-13_46_10.ply
# rosbag2_2023_03_09-13_42_46.png                    rosbag2_2023_03_09-13_44_54_lidar_intensities.png  rosbag2_2023_03_09-13_46_10.png
# rosbag2_2023_03_09-13_44_10_lidar_indices.png      rosbag2_2023_03_09-13_44_54.ply                    rosbag2_2023_03_09-13_46_54_lidar_indices.png
```

### 3a. Initial guess (Automatic)

!!!warning
    SuperGlue is not allowed to be used for commercial purposes. If you are working for profit, use the manual initial guess estimation instead.

Run SuperGlue to find correspondences between LiDAR can camera images. Because SuperGlue requires the upward directions of images are roughtly aligned, add ```---rotate_camera 90``` option to rotate camera images:
```bash
$ ros2 run direct_visual_lidar_calibration find_matches_superglue.py livox_preprocessed --rotate_camera 90
```

Then, run ```initial_guess_auto``` to perform RANSAC-based initial guess estimation:
```bash
$ ros2 run direct_visual_lidar_calibration initial_guess_auto livox_preprocessed
```

![rosbag2_2023_03_09-13_42_46_superglue](https://user-images.githubusercontent.com/31344317/228459350-d1d4c83a-53b7-409f-bddd-d548a08b3c9f.jpg)


### 3b. Initial guess (Manual)

```bash
$ ros2 run direct_visual_lidar_calibration initial_guess_manual livox_preprocessed
```

1. Right click a 3D point on the point cloud and a corresponding 2D point on the image
2. Click ```Add picked points``` button
3. Repeat 1 and 2 for several points (At least three points. The more the better.)
4. Click ```Estimate``` button to obtain an initial guess of the LiDAR-camera transformation
5. Check if the image projection result is fine by changing ```blend_weight```
6. Click ```Save``` button to save the initial guess

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/FTlC9RwEVxY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

### 4. Fine registration

Perform NID-based fine LiDAR-camera registration to refine the LiDAR-camera transformation estimate:
```bash
$ ros2 run direct_visual_lidar_calibration calibrate livox_preprocessed
```

### 5. Calibration result inspection

```bash
$ ros2 run direct_visual_lidar_calibration calibrate livox_preprocessed
```

![Screenshot_20230328_182427](https://user-images.githubusercontent.com/31344317/228191764-837e5fa1-b5d1-4d62-817b-f306473554d5.png)

### 6. Calibration result file

Once the calibration is completed, open ```livox_preprocessed/calib.json``` with a text editor and find the calibration result ```T_lidar_camera: [x, y, z, qx, qy, qz, qw]``` that transforms a 3D point in the camera frame into the LiDAR frame (i.e., ```p_lidar = T_lidar_camera * p_camera```).

```calib.json``` also contains camera parameters, manual/automatic initial guess results (```init_T_lidar_camera``` and ```init_T_lidar_camera_auto```), and some meta data.

<details>
  <summary>calib.json example</summary>

```json
{
  "camera": {
    "camera_model": "plumb_bob",
    "distortion_coeffs": [
      -0.04203564850455424,
      0.0873170980751213,
      0.002386381727224478,
      0.005629700706305988,
      -0.04251149335870252
    ],
    "intrinsics": [
      1452.711762456289,
      1455.877531619469,
      1265.25895179213,
      1045.818593664107
    ]
  },
  "meta": {
    "bag_names": [
      "rosbag2_2023_03_09-13_42_46",
      "rosbag2_2023_03_09-13_44_10",
      "rosbag2_2023_03_09-13_44_54",
      "rosbag2_2023_03_09-13_46_10",
      "rosbag2_2023_03_09-13_46_54"
    ],
    "camera_info_topic": "/camera_info",
    "data_path": "livox",
    "image_topic": "/image",
    "intensity_channel": "intensity",
    "points_topic": "/livox/points"
  },
  "results": {
    "T_lidar_camera": [
      0.023215513184544914,
      -0.049304803782681345,
      -0.0010268378243773314,
      0.002756788930227678,
      0.7121675520572427,
      0.0038417302647440915,
      0.7019936032615696
    ],
    "init_T_lidar_camera_auto": [
      0.01329274206061581,
      -0.055999414521382934,
      0.0033183505131586903,
      0.002471267432195032,
      0.7121558216581672,
      0.0030750358632291534,
      0.7020103437059168
    ]
  }
}

```

</details>

!!!tip
    If you need the transformation in another form (e.g., 4x4 matrix), use the [Matrix converter](https://staff.aist.go.jp/k.koide/workspace/matrix_converter/matrix_converter.html).


## Ouster-camera calibration

**Most of the commands below are the same as those for the Livox dataset except for the options for the preprocessing. See the Livox example for detailed explanation.**

### 1. Download dataset

Download [ouster.tar.gz](https://zenodo.org/record/7777354/files/ouster.tar.gz?download=1) from [zenodo repository](https://zenodo.org/record/7780490) and unzip it:
```bash
$ tar xzvf ouster.tar.gz
```


The ouster dataset contains two ros2 bags, and each bag contains points/image/camera_info topics:
```bash
$ ls ouster
# rosbag2_2023_03_28-16_25_54  rosbag2_2023_03_28-16_26_51

$ ros2 bag info ouster/rosbag2_2023_03_28-16_25_54/
# Files:             rosbag2_2023_03_28-16_25_54_0.db3
# Bag size:          1.2 GiB
# Storage id:        sqlite3
# Duration:          29.14s
# Start:             Mar 28 2023 16:25:54.559 (1679988354.559)
# End:               Mar 28 2023 16:26:23.573 (1679988383.573)
# Messages:          407
# Topic information: Topic: /camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 58 | Serialization Format: cdr
#                    Topic: /image | Type: sensor_msgs/msg/Image | Count: 58 | Serialization Format: cdr
#                    Topic: /points | Type: sensor_msgs/msg/PointCloud2 | Count: 291 | Serialization Format: cdr
```

### 2. Preprocessing

For spinning-type LiDARs, enable dynamic points integration to generate dense point clouds while compensating for the sensor motion.
```bash
# -a : Detect points/image/camera_info topics automatically
# -d : Use dynamic points integrator
# -v : Enable visualization
$ ros2 run direct_visual_lidar_calibration preprocess ouster ouster_preprocessed -adv
```

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/81EqDhUwzXE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

### 3a. Initial guess (Automatic)

!!!warning
    SuperGlue is not allowed to be used for commercial purposes.

```bash
$ ros2 run direct_visual_lidar_calibration find_matches_superglue.py ouster_preprocessed
$ ros2 run direct_visual_lidar_calibration initial_guess_auto ouster_preprocessed
```

### 3b. Initial guess (Manual)

```bash
$ ros2 run direct_visual_lidar_calibration initial_guess_manual ouster_preprocessed
```

### 4. Fine registration

```bash
$ ros2 run direct_visual_lidar_calibration calibrate ouster_preprocessed
```

### 5. Calibration result inspection

```bash
$ ros2 run direct_visual_lidar_calibration calibrate ouster_preprocessed
```

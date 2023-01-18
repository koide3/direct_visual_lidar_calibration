# Usage

## 1. Data collection

## 2. Preprocessing

```bash
$ ros2 run direct_visual_lidar_calibration preprocess --help

preprocess:
  --help                                produce help message
  --data_path arg                       directory that contains rosbags for 
                                        calibration
  --dst_path arg                        directory to save preprocessed data
  --bag_id arg                          specify the bag to use (just for 
                                        evaluation)
  --first_n_bags arg                    use only the first N bags (just for 
                                        evaluation)
  -a [ --auto_topic ]                   automatically select topics
  -d [ --dynamic_lidar_integration ]    create target point cloud from dynamic 
                                        LiDAR data (for velodyne-like LiDARs)
  -i [ --intensity_channel ] arg (=auto)
                                        auto or channel name
  --camera_info_topic arg
  --image_topic arg
  --points_topic arg
  --camera_model arg (=auto)            auto, atan, plumb_bob, fisheye, 
                                        omnidir, or equirectangular
  --camera_intrinsics arg               camera intrinsic parameters 
                                        [fx,fy,cx,cy(,xi)] (don't put spaces 
                                        between values!!)
  --camera_distortion_coeffs arg        camera distortion parameters 
                                        [k1,k2,p1,p2,k3] (don't put spaces 
                                        between values!!)
  --voxel_resolution arg (=0.002)       voxel grid resolution
  --min_distance arg (=1)               minimum point distance. Points closer 
                                        than this value will be discarded
  -v [ --visualize ]                    if true, show extracted images and 
                                        points
```

Example1: Automatically detect image/camera_info/points topics and use the dynamic point integrator for an Ouster LiDAR
```bash
# -a = Automatic topic detection
# -d = Dynamic points integration
# -v = Visualization
ros2 run direct_visual_lidar_calibration preprocess ouster_pinhole ouster_pinhole_preprocessed -a -d -v
```

Example2: Manually specify image/points topics and camera parameters
```bash
ros2 run direct_visual_lidar_calibration preprocess ouster_pinhole ouster_pinhole_preprocessed \
  --camera_model plumb_bob \
  --camera_intrinsics 810.38,810.28,822.84,622.47
  --camera_distortion_coeffs
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



```bash
ros2 run direct_visual_lidar_calibration calibrate ouster_pinhole_preprocessed
```

```bash
ros2 run direct_visual_lidar_calibration viewer ouster_pinhole_preprocessed
```

# Calibration

![Screenshot_20230119_173747](https://user-images.githubusercontent.com/31344317/213393928-c640e0d8-076a-4272-90b3-c67dfab02358.png){: style="height:75%;width:75%"}

**Example data: ~~ouster_pinhole.tar.gz~~ (will be available soon)**

## 1. Preprocessing

```bash
ros2 run direct_visual_lidar_calibration preprocess data_path dst_path
```

| Option                             | Description                                                                     |
|:-----------------------------------|:--------------------------------------------------------------------------------|
| -v [ --visualize ]                 | Show points accumulation process                                                |
| -a [ --auto_topic ]                | Select image/camera_info/points topics automatically                            |
| -d [ --dynamic_lidar_integration ] | Create target point clouds from dynamic LiDAR data (for velodyne-like LiDARs)   |
| -i [ --intensity_channel ]         | "auto" or point intensity channel name                                          |
| --camera_info_topic                | CameraInfo topic name                                                           |
| --image_topic                      | Image topic name                                                                |
| --points_topic                     | PointCloud2 topic name                                                          |
| --camera_model                     | auto, atan, plumb_bob, fisheye, omnidir, or equirectangular                     |
| --camera_intrinsic                 | Camera intrinsic parameters: fx,fy,cx,cy(,xi) (Don't put spaces between values) |
| --camera_distortion_coeffs         | Camera distortion parameters [k1,k2,p1,p2,k3] (Don't put spaces between values) |
| --voxel_resolution                 | Downsampling resolution                                                         |
| --min_distance                     | Minimum point distance. Points closer than this value will be discarded         |

***Example1***: Automatically select image/camera_info/points topics and use the *dynamic point integrator* for an Ouster LiDAR
```bash
# -a = Automatic topic selection
# -d = Dynamic points integration
# -v = Visualization
ros2 run direct_visual_lidar_calibration preprocess ouster_pinhole ouster_pinhole_preprocessed -a -d -v
```

!!!note
    Automatic topic selection requires rosbags to NOT contain multiple image/camera_info/points topics.

!!!note
    ```distortion_model``` of camera info msg must be ```plumb_bob``` or ```fisheye```. For other distortion models, specify the camera name and parameters manually as in Example2.


***Example2***: Manually specify image/points topics and camera parameters
```
ros2 run direct_visual_lidar_calibration preprocess ouster_pinhole ouster_pinhole_preprocessed \
  --image_topic /camera/image \
  --points_topic /os_cloud_node/points \
  --camera_model plumb_bob \
  --camera_intrinsics 810.38,810.28,822.84,622.47
  --camera_distortion_coeffs
```

## 2. Initial guess

### Manual estimation

```bash
ros2 run direct_visual_lidar_calibration initial_guess_manual preprocessed_data_path
```

1. Right click a 3D point on the point cloud and a corresponding 2D point on the image
2. Click ```Add picked points``` button
3. Repeat 1 and 2 for several points (at least three points)
4. Click ```Estimate``` button to obtain an initial guess of the LiDAR-camera transformation
5. Check if the image projection result is fine by changing ```blend_weight```
6. Click ```Save``` button to save the initial guess

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/FTlC9RwEVxY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

### Automatic estimation

!!!warning
    SuperGlue is not allowed to be used for commercial purposes!!

```bash
ros2 run direct_visual_lidar_calibration find_matches_superglue.py preprocessed_data_path
ros2 run direct_visual_lidar_calibration initial_guess_auto preprocessed_data_path
```

!!!note
    SuperGlue requires the upward directions of camera and LiDAR images are roughly aligned. Use ```--rotate_camera``` and ```--rotate_lidar``` options to ensure this. (e.g., ```--rotate_camera 90```)

!!!note
    You can find ```*_superglue.png``` saved in the data directory that represents detected 2D-3D correspondences.
![rosbag2_2022_12_09-11_51_00_superglue](https://user-images.githubusercontent.com/31344317/213388449-e1a80f56-1cc7-45b6-bf0e-239505e3dc24.png)



## 3. Calibration

```bash
ros2 run direct_visual_lidar_calibration calibrate preprocessed_data_path
```

| Option                                              | Description                                                                     |
|:----------------------------------------------------|:--------------------------------------------------------------------------------|
| --registration_type (default = nid_bfgs)            | ```nid_bfgs``` or ```nid_nelder_mead```                                         |
| --nid_bins (default = 16)                           | Number of histogram bins for NID                                                |
| --nelder_mead_init_step (default = 0.001)           | Nelder-mead initial step size                                                   |
| --nelder_mead_convergence_criteria (default = 1e-8) | Nelder-mead convergence criteria                                                |
| --auto_quit                                         | Automatically close the viewer after calibration                                |
| --background                                        | Disable visualization                                                           |

## 4. Result inspection

```bash
ros2 run direct_visual_lidar_calibration viewer preprocessed_data_path
```
![Screenshot_20230119_173547](https://user-images.githubusercontent.com/31344317/213393507-efe30dce-097f-4b65-b91f-56336454991d.png)
# Data collection

## Prerequisite

- The intrinsic parameters of the camera needs to be calibrated.
- The camera and LiDAR are rigidly fixed.


## Data collection steps

- Keep the sensor at rest and start recording Image and PointCloud2 messages. It is a good idea to record CameraInfo as well if it is available.
- **[For a non-repetitive scan LiDAR]** Wait for 10 ~ 15 sec without moving the sensor.
- **[For a spinning LiDAR]** Move the sensor up and down slowly for 5 sec. If your LiDAR has fewer scan lines (e.g., 16 or 32 lines), keep moving it a bit longer (e.g., 10 ~ 15 sec).
- Stop recording.

!!!note
    While the calibration can be performed with only one rosbag at a minimum, we recommend taking several (5 ~ 10) rosbags for better calibration results.

## Keep in mind

- The sensor must be kept at rest at the beginning of each rosbag to ensure that the first point cloud and image are taken at the same pose.
- LiDAR intensity data must exhibit some texture and geometry information. Be aware of that some LiDARs (e.g., Livox Avia) returns invalid intensity values for too close points.
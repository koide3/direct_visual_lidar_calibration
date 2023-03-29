# Data collection

## Prerequisite

- The intrinsic parameters of the camera (i.e., camera matrix and distortion coefficients) need to be calibrated beforehand.
- The camera and LiDAR must be rigidly fixed.


## Data collection steps

- Keep the sensor at rest and start recording ```Image``` and ```PointCloud2``` messages. We recommend recording ```CameraInfo``` as well if it is available.
- **[For a non-repetitive scan LiDAR (Livox)]** Wait for 10 ~ 15 sec without moving the sensor.
- **[For a spinning LiDAR (Ouster)]** Move the sensor up and down slowly for 10 sec. If your LiDAR has fewer scan lines (e.g., 16 or 32 lines), keep moving it a bit longer (e.g., 20 ~ 30 sec).
- Stop recording.

!!!note
    While the calibration can be performed with only one rosbag at a minimum, we recommend taking several (5 ~ 10) rosbags for better calibration results.

Data acquisition example for an Ouster OS1-128:
<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/Urs36qdSQm0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

## Keep in mind

- The sensor must be kept at rest at the beginning of each rosbag to ensure that the first point cloud and image are taken at the same pose.
- LiDAR intensity data must exhibit some texture and geometry information. Be aware of that some LiDARs (e.g., Livox Avia) returns invalid intensity values for too close points.
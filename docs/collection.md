# Data collection

- Keep the sensor at rest and start rosbag recording.
- **[For a non-repetitive scan LiDAR]** Wait for 10 ~ 15 sec without moving the sensor.
- **[For a spinning LiDAR]** Move the sensor up and down slowly for 5 sec. If your LiDAR has fewer scan lines, keep moving it for a bit longer (e.g., 10 ~ 15 sec).
- Stop recording.

!!!note
    While the calibration can be done with only one rosbag, we recommend taking several (5 ~ 10) rosbags for better accuracy.

## Keep in mind

- The sensor must be kept at rest at the beginning of each rosbag to ensure that the first point cloud and image are taken at the same pose.
- LiDAR intensity data must exhibit some texture and geometry information. Be aware of that some LiDARs (e.g., Livox Avia) returns invalid intensity values for too close points.
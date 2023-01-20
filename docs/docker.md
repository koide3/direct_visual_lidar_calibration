# Docker images

We provide ROS1/ROS2 docker images on our docker hub repository:

- [koide3/direct_visual_lidar_calibration:noetic ![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/direct_visual_lidar_calibration/noetic)](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)
- [koide3/direct_visual_lidar_calibration:humble ![Docker Image Size (tag)](https://img.shields.io/docker/image-size/koide3/direct_visual_lidar_calibration/humble)](https://hub.docker.com/repository/docker/koide3/direct_visual_lidar_calibration)

## Pull image

```bash
docker pull koide3/direct_visual_lidar_calibration:humble
```

## Run programs

**Example1**: Run preprocessing
```bash
docker run \
  --rm \
  -v /path/to/input/bags:/tmp/input_bags \
  -v /path/to/save/result:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess -a /tmp/input_bags /tmp/preprocessed
```

**Example2**: Run preprocessing with GUI
```bash
docker run \
  --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v /path/to/input/bags:/tmp/input_bags \
  -v /path/to/save/result:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess -a -v /tmp/input_bags /tmp/preprocessed
```


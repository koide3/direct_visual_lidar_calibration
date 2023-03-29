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
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v /path/to/input/bags:/tmp/input_bags \
  -v /path/to/save/result:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess -a -v /tmp/input_bags /tmp/preprocessed
```

## Examples

See also [Calibration example](example.md) page.

<details>
  <summary>Full commands for Livox-camera calibration example on docker (ROS2)</summary>
```bash
bag_path=$(realpath livox)
preprocessed_path=$(realpath livox_preprocessed)

# Preprocessing
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess -av /tmp/input_bags /tmp/preprocessed

# Initial guess
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed

# Fine registration
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed

# Result inspection
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed
```
</details>

<details>
  <summary>Full commands for Livox-camera calibration example on docker (ROS1)</summary>
```bash
bag_path=$(realpath livox_ros1)
preprocessed_path=$(realpath livox_ros1_preprocessed)

# Preprocessing
docker run \
  -it \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:noetic \
  rosrun direct_visual_lidar_calibration preprocess -av \
  --camera_model plumb_bob \
  --camera_intrinsic 1452.711762456289,1455.877531619469,1265.25895179213,1045.818593664107 \
  --camera_distortion_coeffs -0.04203564850455424,0.0873170980751213,0.002386381727224478,0.005629700706305988,-0.04251149335870252 \
  /tmp/input_bags /tmp/preprocessed

# Initial guess
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:noetic \
  rosrun direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed

# Fine registration
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:noetic \
  rosrun direct_visual_lidar_calibration calibrate /tmp/preprocessed

# Result inspection
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:noetic \
  rosrun direct_visual_lidar_calibration viewer /tmp/preprocessed
```
</details>
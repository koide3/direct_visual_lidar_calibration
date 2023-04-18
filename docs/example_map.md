# Camera-map registration

### 1. Download dataset

Download [faro.tar.gz](https://zenodo.org/record/7780490/files/faro.tar.gz?download=1) from [zenodo repository](https://zenodo.org/record/7780490) and unzip it:
```bash
$ tar xzvf faro.tar.gz
```

```bash
$ ls faro
# camera_params.txt  image00.jpg  map.ply

cat faro/camera_params.txt
# camera_model: plumb_bob
# intrinsic: 2242.45,2242.69,1579.98,1172.42
# distortion: -0.0272569,0.00207913,-0.00411347,-0.00208982,-0.00291759
```

### 2. Preprocessing

```bash
$ ros2 run direct_visual_lidar_calibration preprocess_map \
  --map_path faro/map.ply \
  --image_path faro/image00.jpg \
  --dst_path faro_preprocessed \
  --camera_model plumb_bob \
  --camera_intrinsics 2242.45,2242.69,1579.98,1172.42 \
  --camera_distortion_coeffs -0.0272569,0.00207913,-0.00411347,-0.00208982,-0.00291759
```

### 3. Initial guess

```bash
$ ros2 run direct_visual_lidar_calibration initial_guess_manual faro_preprocessed
```

### 4. Fine registration

```bash
$ ros2 run direct_visual_lidar_calibration calibrate faro_preprocessed
```

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/yTz07B23NXM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>
# 双 MEI 鱼眼相机与 LiDAR 外参求精

利用录制开头固定时长的静止 LiDAR 点云和两张 MEI 鱼眼图像，在外参初值附近分别求精 LiDAR 到 cam0、cam1 的刚体变换。相机内参固定，两台相机独立优化，不需要标定板。

## 原理

优化核心来自 Koide 等人的 [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)。原论文为 *General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox*，ICRA 2023，见 [原始工作](https://github.com/koide3/direct_visual_lidar_calibration#publication)。

处理流程：

1. 根据 IMU header 和 LiDAR 逐点时间，提取开头共同覆盖的固定区间。
2. 检查数据连续性、IMU 运动指标及前后半段点云几何一致性。
3. 过滤无效点和距离范围，体素降采样，读取或选择两张图像。
4. 按初始外参和各自的 MEI 模型筛选可见点，将相机灰度与 LiDAR 反射强度均衡化。
5. 最小化归一化信息距离 NID，分别优化两台相机的六自由度外参。
6. 输出前后外参、实际输入数据和投影对比图。

MEI 模型使用 xi、焦距、主点及径向/切向畸变，将相机坐标中的点投影到鱼眼图像。NID 利用图像亮度与激光反射强度的统计依赖进行配准：

```text
NID = (联合熵 - 互信息) / 联合熵
```

该流程需要可靠的外参初值，不估计内参、时间偏移或两台相机的联合约束。LiDAR 点保留原坐标，不做运动补偿；求精效果通过前后投影检查。

### 静止区间与选帧

```text
t0 = max(首个 IMU header 时间, 首个 LiDAR 逐点时间)
区间 = [t0, t0 + static.duration_sec)
```

duration_sec 指定固定时长，程序检查整段是否满足静止条件，不自动寻找最长静止时长。检查失败即停止，不自动缩短或改用后段。

bag_only 模式要求两张图像的 header 时间都处于该区间。默认选择时间最接近区间中点、两路时间戳完全相同的一对；目标时刻和配对容差可通过 image_selection 配置。图像与 LiDAR/IMU 必须使用同一时间基准。IMU 使用 header，CustomMsg 使用 timebase + offset_time，PointCloud2 使用显式逐点时间；bag 记录时间仅约束读取范围，可与传感器时间不同，不替代选帧或静止区间时间。

外部图像不做时间配对，需要与所选点云对应同一静止姿态。

## 输入

所有业务输入通过 YAML 配置，相对路径以 YAML 所在目录为基准。下列文件是随本仓库发布的模板，未包含用户实际标定值。请复制到自己的配置目录，再填写相机尺寸、内参、畸变、外参初值及输入输出路径；相机参数中的 null 不能直接用于标定。

| 配置 | 输入方式 | 支持的 bag 格式 |
| --- | --- | --- |
| [calibrate_from_bag.example.yaml](../config/calibrate_from_bag.example.yaml) | bag_only：同一 bag 提供 LiDAR、IMU、两路图像 | ROS2 目录或 ROS1 .bag 文件 |
| [calibrate_from_bag_and_images.example.yaml](../config/calibrate_from_bag_and_images.example.yaml) | bag_and_images：bag 提供 LiDAR/IMU，另给两张图像文件 | ROS2 目录或 ROS1 .bag 文件 |

两种模式均支持 ROS1 V2.0 .bag 文件和 ROS2 bag 目录。ROS2 路径填写包含 metadata.yaml 的目录，支持多个 db3 分片；单个 db3 文件不能代替 bag 目录。程序检查实际文件头、索引或元数据，文件扩展名不决定格式。

static.backend 默认 auto：ROS1 优先使用 rosbags 离线读取，ROS2 优先使用 rosbag2_py。ROS1 输入无需启动 roscore、ros1_bridge 或转换为 ROS2；bag 内嵌的 Livox 消息定义用于解码，无需安装 Livox ROS1 消息包。求精继续使用当前 ROS2 工作区的原生 calibrate。损坏文件、缺少依赖或后端与格式冲突会明确报错。

必填数据：

- bag 路径、LiDAR/IMU 话题及预期 LiDAR frame_id。
- bag_only 的两路图像话题，或 bag_and_images 的两张图像路径。
- 每台相机的 MEI 内参、畸变、图像尺寸和初始 4×4 外参。
- 固定静止时长与输出目录。

LiDAR 支持带逐点时间的 PointCloud2 和 Livox CustomMsg；IMU 使用 sensor_msgs/Imu。图像支持 JPEG/PNG CompressedImage，以及 mono8、8UC1、bgr8、rgb8、bgra8、rgba8 的 Image。

图像尺寸必须与内参一致，不隐式缩放、裁剪、旋转或应用 EXIF 方向。可选 mask 与图像同尺寸，非零有效。cam0/cam1 的图像、内参和外参应一一对应。

全部字段、默认值、单位和调整说明见 [完整配置说明](dual_mei_configuration_zh.md)。

## 用法

环境：Ubuntu、ROS2 Humble、已构建的工作区。源码按 `<workspace>/src/direct_visual_lidar_calibration` 放置，依赖和构建步骤见 [编译与使用指南](build_and_usage_zh.md)。以下命令使用 zsh；将工作区和配置目录占位路径替换为自己的路径：

```zsh
cd /path/to/dual_mei_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# 首次使用时，从仓库模板建立自己的配置文件。
mkdir -p /path/to/my_calibration
cp src/direct_visual_lidar_calibration/config/calibrate_from_bag.example.yaml /path/to/my_calibration/calibrate_from_bag.yaml
cp src/direct_visual_lidar_calibration/config/calibrate_from_bag_and_images.example.yaml /path/to/my_calibration/calibrate_from_bag_and_images.yaml
```

Fast-Dual360 工作区根目录中已有的同名 YAML 是该工作区的实际运行配置，不属于此 fork 的模板。已有配置时直接使用自己的文件，不要用模板覆盖。

编辑所选 YAML 的路径、相机参数、static.duration_sec 和 output.directory，然后运行：

```zsh
# 图像来自 bag
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py --config /path/to/my_calibration/calibrate_from_bag.yaml

# 单独提供两张图像
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py --config /path/to/my_calibration/calibrate_from_bag_and_images.yaml
```

两份 YAML 是独立配置，不自动同步输入或参数。输出目录必须不存在或为空。

使用 ROS1 合包时，在同一份 bag_only YAML 中设置以下字段，其余相机、静止和求精参数照常填写：

```yaml
input:
  mode: bag_only
  bag_path: /path/to/merged.bag
  lidar_topic: /livox/lidar
  imu_topic: /livox/imu
  images:
    cam0: {topic: /cam0/image_raw/compressed}
    cam1: {topic: /cam1/image_raw/compressed}
static:
  backend: auto
  duration_sec: 3.0
```

这是配置片段；完整配置还需相机内外参及新的 output.directory。

完整文件可从 [bag 模板](../config/calibrate_from_bag.example.yaml) 或 [外部图像模板](../config/calibrate_from_bag_and_images.example.yaml) 建立；image_selection.target_offset_sec 等可选字段允许的 null 含义见 [配置说明](dual_mei_configuration_zh.md)，不需要将所有 null 都替换成数值。

仅检查静止区间，不写结果：

```zsh
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py --config /path/to/my_calibration/calibrate_from_bag.yaml --check-static-only
```

增加 --prepare-only 可只保存准备好的图像、点云、before 图及 prepared.yaml，不运行优化、不生成最终外参；同样需要新的或空输出目录。

### 查看前后投影

将 /path/to/result 替换为 output.directory：

```zsh
# 交互查看，需要图形显示环境
ros2 run direct_visual_lidar_calibration visualize_dual_mei.py --result /path/to/result

# 无图形显示环境中重新导出
ros2 run direct_visual_lidar_calibration visualize_dual_mei.py \
  --result /path/to/result --render-only --output /tmp/dual_mei_review
```

| 操作 | 功能 |
| --- | --- |
| 缩放滑块或 + / - | 缩放 |
| 左键拖动 | 平移 |
| 0 / 1 | 切换相机 |
| B / A | 切换优化前 / 后 |
| 点半径、透明度滑块 | 调整投影点显示 |
| R / S / Q 或 Esc | 重置 / 保存投影图 / 退出 |

可增加 --config /path/to/display.yaml，覆盖 visualization 参数。查看器只依赖结果目录，不重新读取 bag 或运行优化；调大 point_radius_px 可让点更明显。

### 构建与验证

依赖安装、工作区构建和运行入口见 [编译与使用指南](build_and_usage_zh.md)。`scripts/setup_ros2.zsh` 和环境 hook 按上述工作区布局解析 `deps/` 中的本地依赖；移动工作区后需要重新配置和构建。

`tests/test_*.py` 是功能测试；`tests/verify_*.py` 是验收辅助脚本，其中 `verify_dual_mei_delivery.py` 和 `verify_scaled_input.py` 依赖 Fast-Dual360 本地的 `data/0908` 结果、`.task_artifacts/0909_extrinsics` 基线或指定内参文件。这些数据不随本 fork 发布，不能仅凭克隆仓库复现对应验收。`tests/headless_smoke_result.md` 保存一次历史环境的验证记录。

## 输出

无 mask 的成功结果共 9 个文件：

```text
output.directory/
├── extrinsics.yaml
├── cam0/
│   ├── image.jpg
│   ├── lidar.ply
│   ├── overlay_before.png
│   └── overlay_after.png
└── cam1/
    ├── image.jpg
    ├── lidar.ply
    ├── overlay_before.png
    └── overlay_after.png
```

| 文件 | 内容 |
| --- | --- |
| extrinsics.yaml | 两台相机的初始/最终外参、内参、输入来源、选帧时间、静止区间、必要参数及简短求解状态 |
| camN/image.jpg 或 image.png | 实际采用的图像；JPEG/PNG 压缩载荷保留，普通 Image 保存为无损 PNG |
| camN/lidar.ply | 交给该相机优化器的 float32 XYZI 点云，坐标系仍为 LiDAR |
| camN/overlay_before.png / overlay_after.png | 初始/最终外参对应的投影图 |
| camN/mask.png（可选） | 配置 mask 时额外保存 |

外参约定：

```text
p_camera = T_cam_lidar @ p_lidar
```

T_cam_lidar_initial 为初值，T_cam_lidar 为求精结果，均表示 LiDAR → camera，平移单位米。

PLY 的 intensity 为该相机点集按经验 CDF 均衡化后的反射强度，范围 0–1。投影图使用保存的点云，前后共用着色范围，同一像素显示最近点。

只有两台相机成功且投影有效才发布正式结果。临时文件自动清理，不保存完整迭代日志；错误输出到终端并返回非零退出码。

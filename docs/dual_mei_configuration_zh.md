# YAML 配置说明

运行方式和输入输出见 [双 MEI 外参求精说明](dual_mei_static_zh.md)。配置起点为 [bag 模板](../config/calibrate_from_bag.example.yaml) 和 [外部图像模板](../config/calibrate_from_bag_and_images.example.yaml)。模板中的相机尺寸、内参、畸变和外参初值为 null，须填写实际标定值；其他可选字段的 null 按下表含义处理。用户实际运行配置应从模板另存，不用其替换已有设备配置。

省略字段时使用下表默认值，未知字段会报错。schema_version: 2 为常用格式；版本 1 保留外部图像兼容。所有相对路径以 YAML 所在目录为基准。

## 1 输入与模式

| 字段 | 默认/取值 | 含义与使用方法 |
| --- | --- | --- |
| schema_version | 必填，2 | 配置格式版本，不是相机/ROS 版本 |
| input.mode | 必填：bag_only / bag_and_images | 决定两张图像从 bag 读取还是从文件读取 |
| input.bag_path | 必填 | ROS1 V2.0 .bag 文件或包含 metadata.yaml 的 ROS2 bag 目录；两种模式均支持，且都需要 LiDAR/IMU |
| input.lidar_topic | /livox/lidar | 点云消息话题 |
| input.imu_topic | /livox/imu | 静止检查使用的 IMU 话题 |
| input.lidar_frame | livox_frame | 预期 LiDAR 坐标系名称；正常标定时与消息 frame_id 比较，不执行坐标变换 |
| input.images.camN.topic | bag_only 必填 | 该相机的 Image / CompressedImage 话题，两相机与 LiDAR/IMU 话题必须各不相同 |
| input.images.camN.image_path | bag_and_images 必填 | 该相机的外部图像路径；本模式配置中不放 topic |

bag_only 的 input.images 中只放 topic；bag_and_images 中只放 image_path，且不放 image_selection。旧配置中多余的另一模式字段仍可读取以保留兼容，但新文件不再混写。

## 2 图像选帧（仅 bag_only）

| 字段 | 默认 | 含义与调整影响 |
| --- | --- | --- |
| image_selection.target_offset_sec | null | 相对 t0 的目标秒偏移；null 使用中点；范围为 0 ≤ offset < duration_sec |
| image_selection.max_pair_delta_sec | 0.0 秒 | 两相机时间差容限；0 为严格相同。若数据本来不完全同步，可按已知采集误差填写，不会因此校正时间 |

外部模式直接用给定图像，不使用这两个参数。

## 3 静止区间、时间格式与点云过滤

| 字段 | 默认值；示例取值 | 含义与调整影响 |
| --- | --- | --- |
| static.duration_sec | 1.0 秒 | 请求积分并检查的固定时长；更长可积累更多点，但整段都必须满足检查 |
| static.backend | auto | auto 对 ROS1 优先 rosbags、缺依赖时尝试原生 rosbag；对 ROS2 优先 rosbag2、缺依赖时尝试 rosbags。rosbags 可读两种格式；rosbag2 仅 ROS2，rosbag 仅 ROS1；显式格式冲突直接报错，损坏数据不触发后端回退 |
| static.pointcloud_time_field | auto；示例 timestamp | PointCloud2 内每点时间字段名；显式填写避免字段歧义 |
| static.pointcloud_time_unit | auto；示例 ns | 每点时间单位：s、ms、us、ns 或 auto；Livox CustomMsg 使用其 timebase + offset_time |
| static.pointcloud_time_reference | auto；示例 absolute | absolute 为绝对传感器时间；relative 为相对消息 header 的偏移；必须符合数据实际编码 |
| static.acceleration_scale | 9.80665 | 原始 IMU 加速度乘数：g 单位乘 9.80665，已经是 m/s² 则填 1.0；不是调整检测松紧的参数 |
| static.min_range_m | 0.1 米；示例 1.0 | 排除离 LiDAR 过近的点；设置为 1.0 时排除 1 米以内的点 |
| static.max_range_m | 100.0 米 | 排除超过此 LiDAR 距离的点 |
| static.min_reflectivity | 0.0 | 原始反射强度下限；发生在归一化之前，单位随点云数据定义 |
| static.min_points | 100 | 静止区间裁剪与过滤后最低总点数；还没进入每相机拆分 |
| static.max_read_record_sec | 10.0 秒 | 从首个相关传感器记录开始的读取上限；必须 ≥ duration_sec，还要覆盖起点差和末端边界样本 |
| static.pointcloud_max_frame_span_sec | 1.0 秒 | 单条 PointCloud2 的逐点时间跨度上限，帮助识别时间格式/单位异常 |

pointcloud_time_* 的 auto 是时间字段/编码识别，**不是自动检测静止时长**。这些字段只用于 PointCloud2；Livox CustomMsg 始终使用整数纳秒 timebase + offset_time。

IMU header、图像 header 和 LiDAR 逐点时间应处于同一传感器时间基准。max_read_record_sec 约束 bag 记录时间的读取范围，不要求 record 与 header 相等，不进行重新同步或改写时间。ROS2 绝对 FLOAT64 纳秒可能有量化误差，程序沿用其边界保护规则。

## 4 IMU 静止与连续性检查（高级，通常保持默认）

| 字段 | 默认 | 含义与调整影响 |
| --- | --- | --- |
| static.window_sec | 0.25 秒 | IMU 检查窗口长度；区间更短时使用较短的整个区间 |
| static.window_step_sec | 0.125 秒 | 滑动步长，不能大于窗口长度；更小会增加重叠检查 |
| static.gyro_p95_max_rad_s | 0.03 rad/s | 窗口内角速度模长的 95 分位上限，约 1.72°/s；调大放宽旋转检查 |
| static.accel_std_max_m_s2 | 0.196133 m/s² | 三轴加速度标准差组成的向量模长上限；看变化程度，不是把带重力的加速度模长与零比较 |
| static.imu_max_gap_sec | 0.03 秒 | 跨越所选区间的相邻 IMU 时间最大允许间隔，含边界覆盖检查 |
| static.lidar_max_gap_sec | 0.25 秒 | LiDAR 数据包逐点时间覆盖区间之间允许的最大空缺；不是逐点扫描间隔 |
| static.min_imu_samples_per_window | 10 | 每个窗口至少需要多少条 IMU 样本，太少则无法可靠做统计 |

这些值是接受/拒绝数据的阈值，不控制外参优化的收敛精度。出现静止检查失败时，应先看数据、单位和运动情况；调大阈值会放宽验收。

## 5 LiDAR 几何静止检查（高级，通常保持默认）

| 字段 | 默认 | 含义与调整影响 |
| --- | --- | --- |
| static.geometry_check_enabled | true | 开启前后半段点云几何检查；false 会跳过这一检查 |
| static.geometry_voxel_size_m | 0.08 米 | 几何检查使用的降采样尺度，与正式优化的 pointcloud.voxel_size_m 分开 |
| static.geometry_max_points | 12000 | 每半段用于几何检查的点数上限，至少 100 |
| static.geometry_max_correspondence_m | 0.30 米 | ICP 匹配的最近邻距离门限 |
| static.geometry_min_correspondence_ratio | 0.30 | 几何对应点比例下限；过低表示有效重叠不足 |
| static.geometry_max_translation_m | 0.03 米 | 前后半段估计平移上限；这是运动筛查阈值，不是最终外参平移修正上限 |
| static.geometry_max_rotation_deg | 0.5° | 前后半段估计旋转上限，同样不是外参优化步长限制 |
| static.geometry_max_rmse_m | 0.03 米 | ICP 最终点到面残差 RMSE 上限 |
| static.geometry_max_condition_number | 10000 | 归一化信息矩阵条件数上限，用来拒绝不能可靠约束六自由度的退化场景 |

## 6 每台相机 cameras.cam0 / cameras.cam1

| 字段 | 默认/要求 | 含义与使用方法 |
| --- | --- | --- |
| model | 必填 mei | 使用 MEI 统一相机模型 |
| width / height | 必填，像素 | 输入图像实际宽高，必须匹配文件/消息尺寸 |
| fx / fy | 必填，像素 | 横向/纵向焦距，必须 > 0 |
| cx / cy | 必填，像素 | 主点像素坐标 |
| xi | 必填，≥ 0 | MEI 模型参数，无量纲；来自相机内参标定 |
| distortion.k1 / k2 / k3 | 五个畸变参数均须显式填写 | 径向畸变系数，无量纲；不使用的项填 0 |
| distortion.p1 / p2 | 同上 | 切向畸变系数，无量纲 |
| max_theta_deg | 95° | 相对相机 +Z 光轴的最大夹角，是半视角；95° 对应 190° 总角度，实际还受模型、图像边界和 mask 限制 |
| mask_path | 不使用 mask | 可选灰度图路径，与输入图同尺寸；非零有效、零无效；省略即可，不根据黑色像素自动推断 |
| T_cam_lidar | 必填 4×4 | LiDAR → 该相机的外参初值；平移米，旋转正交且行列式 +1，末行 [0,0,0,1] |

这些内参在求精中保持固定。cam0/cam1 分别填写自己的内参与初值，不因双鱼眼布局而自动补一个 180° 旋转。

## 7 点云准备与求解器

| 字段 | 默认 | 含义与调整影响 |
| --- | --- | --- |
| pointcloud.voxel_size_m | 0.02 米 | 优化前统一降采样；每体素取坐标/强度均值；0 关闭。小体素保留更多点，也增加计算量 |
| pointcloud.split_margin_px | 8 像素 | 按初值为每相机筛点时，在图像边缘向内留出的余量；增大后边缘点更少，不是放大投影点 |
| pointcloud.min_camera_points | 500 | 每相机准备点数及前后有效投影点数下限；过少就失败 |
| solver.max_outer_iterations | 10 | 外层可见点更新/求解循环上限；是上限，不保证执行满或必定收敛 |
| solver.max_inner_iterations | 256 | 每次内层数值优化的最大迭代数 |
| solver.num_threads | 2 | 传给原生求解进程的 OMP_NUM_THREADS；控制 OpenMP 并行度，不是相机数量 |
| solver.calibrate_executable | null | 默认通过已 source 的 ROS 环境寻找 calibrate；可显式填写可执行文件路径用于调试 |

solver 分区在两份示例模板中省略，等效于上述默认值。需要覆盖时自行加回对应字段，例如：

```yaml
solver:
  max_outer_iterations: 10
  max_inner_iterations: 256
  num_threads: 4
```

## 8 输出与显示

| 字段 | 默认 | 含义与调整影响 |
| --- | --- | --- |
| output.directory | 正常运行必填 | 正式结果目录，必须不存在或为空 |
| visualization.point_radius_px | 3 像素 | 投影圆点半径，允许 1–32；调大让点更明显 |
| visualization.alpha | 0.8 | 点层叠加权重，范围 0–1；0 只见原图，1 点颜色最明显 |
| visualization.max_points | 0 | 绘制点数上限，0 不额外截断；只影响显示，不影响优化输入 |
| visualization.color_by | range | range 按到相机原点的距离着色；intensity 按保存的均衡化反射强度着色 |
| visualization.color_range | null | null 在首次导出时取前后有效点共同的 5%–95% 范围并保存；也可手填 [min,max]，范围外颜色饱和；range 单位米，intensity 为 0–1 |

同一像素存在多个投影点时显示最近点，因此“绘制点数”通常少于 PLY 点数。前后图共用着色范围，独立查看器默认复用保存的范围，便于比较。

# 双 MEI 标定：编译、依赖与 ROS1/ROS2 使用

本文对应当前 Ubuntu 22.04 / ROS2 Humble 工作空间。支持 ROS1 bag 输入，并不意味着需要另编译一套 ROS1 标定程序：当前双 MEI 管线对两种 bag 都使用宿主机上的同一套 Humble 原生求解器。

## 1. 现有远端机器：直接编译

在 Mac 上连接远端，再在远端 zsh 中执行：

```zsh
ssh bling
cd /home/chenyu/ws_chenyu/recon/Fast-Dual360/direct_visual_lidar_calibration_ros2_ws
source src/direct_visual_lidar_calibration/scripts/setup_ros2.zsh

CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DBUILD_STATIC_MEI_TESTS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.zsh
```

`setup_ros2.zsh` 加载 Humble、工作空间 `deps/install` 和 `deps/system-root/usr` 的 CMake/动态库路径，并在已有安装时加载 `install/setup.zsh`。新终端需要重新加载。使用 `--base-paths src` 只发现项目源码；`deps/COLCON_IGNORE` 也明确排除了外部依赖。

这里的编译命令使用已经准备好的第三方依赖，不会自动下载、编译或安装 GTSAM、Ceres、Iridescence。修改 C++ 后重新编译；修改 YAML 只需重新运行，无需编译。

## 2. 运行：ROS1 与 ROS2 使用同一入口

| 输入 | YAML 的 input.bag_path | static.backend | 构建和运行环境 |
| --- | --- | --- | --- |
| ROS1 bag | 具体 `.bag` 文件 | `auto` | 宿主机 Humble，与 ROS2 输入相同 |
| ROS2 bag | 含 `metadata.yaml` 的 bag 目录 | `auto` | 宿主机 Humble |

ROS1 通过 rosbags 离线读取内嵌消息定义；不需要 roscore、Noetic 容器、ros1_bridge 或先转包。ROS2 默认通过 rosbag2_py 读取，使用 Humble 环境。

先编辑完整 YAML：

- `input.bag_path`：对应实际格式的输入路径。
- `input.images` 或外部图像路径：取决于 `input.mode`。
- `cameras`：实际 MEI 内参、尺寸、畸变和 LiDAR 到相机的外参初值。
- `static.duration_sec`：开头固定静止区间的时长。
- `output.directory`：新的或空结果目录。

当前 Fast-Dual360 工作空间中的 `calibrate_from_bag.yaml` 已含设备参数，默认输入仍是 ROS2 合包，输出指向已有结果；重跑前必须设置新的输出目录。它不会因为 `backend: auto` 自动选择另一个 ROS1 文件。

当前 0909 数据可分别填写：

```yaml
# ROS1
input:
  bag_path: /home/chenyu/ws_chenyu/recon/data/0909/livo_w_insta_ros1/merged.bag
static:
  backend: auto
```

```yaml
# ROS2
input:
  bag_path: /home/chenyu/ws_chenyu/recon/data/0909/livo_w_insta_ros2/merged_bag
static:
  backend: auto
```

以上只是要修改的字段，不是可替代完整配置的独立文件。

在已加载环境的工作空间中：

```zsh
# 先检查静止区间，不写标定结果
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py \
  --config ./calibrate_from_bag.yaml --check-static-only

# bag 内含两路图像
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py \
  --config ./calibrate_from_bag.yaml

# bag 提供 LiDAR/IMU，另给两张外部图像
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py \
  --config ./calibrate_from_bag_and_images.yaml
```

两份 YAML 是独立配置，应分别检查路径、相机参数、静止时长和输出目录。

查看已有结果，无需重新运行标定：

```zsh
ros2 run direct_visual_lidar_calibration visualize_dual_mei.py \
  --result /home/chenyu/ws_chenyu/recon/data/0909/extrinsic_calibration_ros1
```

交互查看需要图形显示环境。无显示时可导出投影：

```zsh
ros2 run direct_visual_lidar_calibration visualize_dual_mei.py \
  --result /home/chenyu/ws_chenyu/recon/data/0909/extrinsic_calibration_ros1 \
  --render-only --output /tmp/dual_mei_review
```

完整功能见 [双 MEI 说明](dual_mei_static_zh.md)，YAML 字段见 [配置说明](dual_mei_configuration_zh.md)。

## 3. deps/install 与 GTSAMConfig.cmake 是什么

GTSAM 是本项目使用的第三方 C++ 数学库，提供位姿/几何和优化相关能力。`deps/install` 是这些依赖在工作空间内的安装前缀，不是标定结果目录，也不是 ROS 包源码目录。

```text
deps/src/gtsam/                       GTSAM 源码
deps/build/<构建目录>/                CMake 缓存、编译中间文件
deps/install/include/gtsam/           安装后的头文件
deps/install/lib/libgtsam.so*         安装后的动态库
deps/install/lib/cmake/GTSAM/         供其他 CMake 工程查找 GTSAM 的配置
```

项目 CMakeLists.txt 的 `find_package(GTSAM REQUIRED)` 读取 `GTSAMConfig.cmake` 与导出 targets，定位头文件、库和依赖。它不是需要手工填写的业务配置。

GTSAM 自己的 CMake 配置阶段生成这些文件，安装阶段将它们写入安装前缀。主项目的 `colcon build` 消费这些文件，不会自动重新生成外部 GTSAM 安装。

当前 GTSAMConfig.cmake 同时包含 build-tree 与 install-tree 两条分支。旧源码绝对路径只出现在未启用的 build-tree 分支；当前安装分支通过配置文件自身位置计算 include 路径。因此，文件中出现旧目录名本身不代表当前 GTSAM 安装失效，也不代表工作空间改名后一定要重编译所有依赖。

## 4. 当前依赖版本与需要重建时的区别

2026-09-09 核查的宿主机安装：

| 依赖 | 当前版本/源码提交 |
| --- | --- |
| GTSAM | 4.2a9，`c57988fe554e7213c77fe379c1d7c483de26ad33` |
| Ceres | 2.1.0，`f68321e7de8929fbcdb95dd42877531e64f72f66` |
| Iridescence | v0.1.9-5，`b1478430cbdee0028267fc7506421107133a96fa` |
| rosbags | 0.11.3 |
| Python | 3.10.12 |

项目背景中 Docker Noetic 镜像的 GTSAM 4.0.3 属于另一套环境，不是当前宿主机标定工作空间中的 GTSAM 版本。

以下分清三种情况：

1. **只修改标定代码**：按第 1 节运行 colcon。
2. **GTSAM 安装缺失或需要换版本**：单独配置、编译、安装 GTSAM，再构建主项目。
3. **移动整个工作空间**：主项目 build/install 含绝对路径，应重新生成；依赖安装先检查 CMake 导出、动态库加载和资源路径，需要时再重建。

现有 `deps/build` 有更早目录留下的缓存，不要直接沿用其中的构建路径。需要重建 GTSAM 时，使用新的空构建目录。下面根据现有源码和构建选项整理，**本次没有实际重建 GTSAM**：

```zsh
# 在工作空间根目录，deps/src/gtsam 已存在且为上表源码版本
calib_ws_root="$PWD"
cmake -S "$calib_ws_root/deps/src/gtsam" \
  -B "$calib_ws_root/deps/build/gtsam-rebuild" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$calib_ws_root/deps/install" \
  -DCMAKE_INSTALL_RPATH="$calib_ws_root/deps/install/lib" \
  -DBUILD_SHARED_LIBS=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF

cmake --build "$calib_ws_root/deps/build/gtsam-rebuild" --parallel 2
cmake --install "$calib_ws_root/deps/build/gtsam-rebuild"
```

依赖修改后重新加载环境，再按第 1 节构建主项目。无需手改 GTSAMConfig.cmake。上面假定 Eigen、Boost 等 GTSAM 的构建依赖已就绪。

## 5. 从 Git 获取源码与首次环境准备

只 clone 本 fork 时，维持下面的工作空间布局：

```text
<workspace>/
├── src/direct_visual_lidar_calibration/
├── deps/        # 单独准备，未提交 Git
├── build/       # colcon 生成
├── install/     # colcon 生成
└── log/         # colcon 生成
```

```zsh
mkdir -p ~/ws_dual_mei/src
git clone --recurse-submodules -b fast-dual360 \
  git@github.com:chenyuwang2020/direct_visual_lidar_calibration.git \
  ~/ws_dual_mei/src/direct_visual_lidar_calibration
```

已有 Fast-Dual360 外层仓库时，应通过外层 `git submodule update --init --recursive` 获取其固定的源码版本。

Git 保存源码、模板、文档与子模块版本，不保存本地 `deps/install`、数据、构建产物或真实结果。干净机器需要先准备 Ubuntu 22.04 / Humble、colcon、编译工具、PCL、OpenCV、Boost、Eigen、Ceres 2.1、GTSAM、Iridescence、GLFW/OpenGL，以及 numpy/scipy/OpenCV/PyYAML/rosbags 等 Python 依赖，不能只 clone 后直接运行 colcon。

首次安装时优先复用上表已核实的源码版本。现有依赖源码来源：

- [GTSAM](https://github.com/borglab/gtsam)
- [Ceres](https://github.com/ceres-solver/ceres-solver)
- [Iridescence](https://github.com/koide3/iridescence)

本次验证的是现有 Ubuntu 宿主机的增量构建，没有在空白系统重做完整依赖安装。不要把本机已安装环境当成由 Git 自动恢复。

## 6. ROS1 原生构建与其他工具

当前源码保留上游根据 `ROS_VERSION` 选择 catkin/ament 的分支，但本项目当前双 MEI 交付使用 Humble。若专门需要构建和运行原生 ROS1 节点，应在独立 Noetic/catkin 工作空间准备匹配依赖并另行验证；不要在当前目录混用 catkin 与 colcon 的构建产物。

| 工作 | 是否需要手动编译 ROS 工作空间 |
| --- | --- |
| 双 MEI 标定 | 需要；当前统一用 Humble/colcon，输入可为 ROS1 或 ROS2 bag |
| Insta360/Livox 同步合包 | 不需要 colcon；由 shell/Python 入口运行，JPEG 辅助 C 源码按需构建缓存 |
| Livox ROS1→ROS2 转包 | 不需要 colcon；Python + rosbags 离线运行 |
| 原生 ROS1 节点/播放 | 使用 Noetic 环境；与离线读取 ROS1 bag 是不同事项 |

## 7. 验证命令与本次结果

```zsh
ctest --test-dir build/direct_visual_lidar_calibration --output-on-failure
python3 -m unittest discover \
  -s src/direct_visual_lidar_calibration/tests -p 'test_*.py'
ros2 run direct_visual_lidar_calibration calibrate_dual_mei.py --help
ros2 run direct_visual_lidar_calibration visualize_dual_mei.py --help
```

2026-09-09 本次 Git 纳管前实跑：colcon 增量构建通过；原生 mei_validation 1/1 通过；Python 测试 63 项通过；两个安装入口帮助命令通过。本次没有重跑完整真实数据标定，也没有重建第三方依赖。

`test_*.py` 使用自动生成的小数据执行回归；`verify_*.py` 是历史真实数据验收脚本，部分依赖本地数据路径及 `.task_artifacts`，不属于 clone 后可无数据直接执行的通用测试。

## 8. 首次准备或重建全部依赖的参考命令

本节基于当前源码版本及已有构建选项整理，未在空白机器执行整套安装。当前远端依赖已可用，日常编译不需要运行本节。

以已经安装 Humble 的 Ubuntu 22.04 为前提，基础依赖可通过系统包准备：

```zsh
sudo apt update
sudo apt install \
  build-essential cmake git \
  libboost-all-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
  libsuitesparse-dev liblapack-dev libfmt-dev libopencv-dev libpcl-dev \
  libglfw3-dev libglm-dev libglew-dev libpng-dev libjpeg-dev libgl1-mesa-dev \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-venv \
  python3-numpy python3-scipy python3-opencv python3-yaml \
  ros-humble-ament-cmake-auto ros-humble-ament-cmake-python \
  ros-humble-rclcpp ros-humble-rclpy ros-humble-sensor-msgs \
  ros-humble-cv-bridge ros-humble-pcl-ros \
  ros-humble-rosbag2-cpp ros-humble-rosbag2-storage \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins \
  ros-humble-rosidl-runtime-py
```

当前远端的 GLFW/GLM 安装于 `deps/system-root/usr`，来自解包的 Ubuntu 软件包；干净机器直接安装上述系统开发包即可，不要求复制该解包目录。

在工作空间根目录获取固定版本的依赖。以下 clone 命令仅用于对应目录尚不存在的情况；已有正确源码时跳过：

```zsh
mkdir -p deps/src
git clone https://github.com/ceres-solver/ceres-solver.git deps/src/ceres-solver
git -C deps/src/ceres-solver checkout f68321e7de8929fbcdb95dd42877531e64f72f66

git clone https://github.com/borglab/gtsam.git deps/src/gtsam
git -C deps/src/gtsam checkout c57988fe554e7213c77fe379c1d7c483de26ad33

git clone https://github.com/koide3/iridescence.git deps/src/iridescence
git -C deps/src/iridescence checkout b1478430cbdee0028267fc7506421107133a96fa
git -C deps/src/iridescence submodule update --init --recursive

git -C src/direct_visual_lidar_calibration submodule update --init --recursive

python3 -m venv --system-site-packages deps/python
source deps/python/bin/activate
python3 -m pip install 'numpy==1.24.4' 'rosbags==0.11.3'
```

使用这个 Python 环境时，每个新终端先激活 `deps/python/bin/activate`，再加载工作区环境。系统 ROS Python 模块通过 `--system-site-packages` 和 Humble 环境可见。

构建 Ceres：

```zsh
source /opt/ros/humble/setup.zsh
calib_ws_root="$PWD"
cmake -S deps/src/ceres-solver -B deps/build/ceres-current \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$calib_ws_root/deps/install" \
  -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF -DCUDA=OFF \
  -DSUITESPARSE=ON -DCXSPARSE=ON -DEIGENSPARSE=ON -DLAPACK=ON \
  -DGFLAGS=ON -DMINIGLOG=OFF
cmake --build deps/build/ceres-current --parallel 2
cmake --install deps/build/ceres-current
```

然后按第 4 节的 GTSAM 命令配置、编译并安装。

构建 Iridescence；`deps/system-root/usr` 不存在时将由系统路径提供 GLFW/GLM：

```zsh
cmake -S deps/src/iridescence -B deps/build/iridescence-current \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$calib_ws_root/deps/install" \
  -DCMAKE_PREFIX_PATH="$calib_ws_root/deps/install;$calib_ws_root/deps/system-root/usr" \
  -DCMAKE_INSTALL_RPATH="$calib_ws_root/deps/install/lib;$calib_ws_root/deps/system-root/usr/lib/x86_64-linux-gnu" \
  -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXT_TESTS=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF
cmake --build deps/build/iridescence-current --parallel 2
cmake --install deps/build/iridescence-current
```

依赖安装完成后回到第 1 节构建主项目，并执行第 7 节的检查。构建目录只能在同一源码/安装路径下复用；目录发生迁移时换用新的构建目录。

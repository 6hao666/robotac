# 环境与构建

## 基线环境

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic |
| Python | Python 3.8 |
| 构建工具 | catkin、CMake、GCC |
| 飞控接口 | PX4 与 MAVROS 1.x |

macOS 可以执行源码检查和不依赖 ROS 的单元测试，但 ROS Noetic 构建和仿真必须在
Ubuntu 20.04 上完成。

## 首次安装

```bash
cd ~/robotac_ws
./tools/install_ubuntu20.sh
```

该脚本安装 apt 与 rosdep 依赖，并构建系统级 `Livox-SDK2` 和 `apriltag`。脚本不会启动 ROS
节点，不连接飞机，也不修改 PX4 参数。第三方库和 catkin 默认使用 2 个并行任务，避免在
内存受限的主机上触发 OOM。资源充足时可显式设置 `ROBOTAC_BUILD_JOBS`：

```bash
ROBOTAC_BUILD_JOBS=4 ./tools/install_ubuntu20.sh
ROBOTAC_BUILD_JOBS=4 ./tools/test_02_build.sh
```

## 首次构建

```bash
./tools/test_02_build.sh
source devel/setup.bash
```

构建成功后应能发现以下项目包：

```bash
rospack find robotac_bringup
rospack find robotac_localization
rospack find robotac_examples
rospack find robotac_servo
```

## 增量构建

```bash
./tools/build.sh
source devel/setup.bash
```

修改 Python、launch 或配置后仍应重新执行源码检查。修改 `package.xml`、CMake 或消息依赖后，
必须重新执行完整构建。

## 离线测试

```bash
./tools/test_01_source.sh
./tools/test_03_unit.sh
./tools/test_04_simulation.sh
```

各工具检查的内容如下：

| 工具 | 检查范围 |
| --- | --- |
| `test_01_source.sh` | Python 语法、行数、XML、YAML、JSON、路径、接口、文档链接 |
| `test_02_build.sh` | catkin 构建和项目包发现 |
| `test_03_unit.sh` | 坐标、Tag 筛选、航点解析、定位检查、舵机协议 |
| `test_04_simulation.sh` | 假飞控下的启动前检查、飞行示例、停止和故障降落 |
| `test_05_hardware_readonly.sh` | 真实设备与 ROS 数据只读检查 |

`test_05_hardware_readonly.sh` 不属于离线测试，不应在容器或无设备环境执行。

## 常见构建问题

### 未找到 ROS Noetic

确认 `/opt/ros/noetic/setup.bash` 存在，并在 Ubuntu 20.04 执行。不得用其他 ROS 发行版的
环境变量继续构建。

### rosdep 未初始化

```bash
sudo rosdep init
rosdep update
```

系统已初始化时不重复执行 `rosdep init`。

### 找不到 Livox SDK 或 apriltag

重新执行 `tools/install_ubuntu20.sh`，检查 `cmake --install` 与 `ldconfig` 是否成功。不要在
第三方源码目录中临时复制动态库。

### 构建目录来自其他机器

`build/` 和 `devel/` 不应跨机器同步。删除本机生成目录后重新构建；该操作不得影响 `src/`
和未提交源码。

# Hesai JT 系列 ROS1 驱动（含 timeshare）

本分支 `JT` 基于禾赛官方 `HesaiLidar_ROS_2.0`（v2.0.12，SDK 2.0 子模块），
并移植了旧版 `HesaiLidar_General_ROS_timeshare` 的 timeshare 机制。

## 1. 目录

```text
HesaiLidar_ROS_2.0/           # ROS1 驱动源码（上游结构）
├── config/
│   ├── config.yaml           # 上游默认配置（UDP 示例）
│   └── jt16_serial.yaml      # 新增：JT16 串口配置
├── launch/
│   ├── start.launch          # 上游（含 rviz）
│   └── jt16_serial.launch    # 新增：JT16 串口无界面启动
├── src/
│   ├── manager/
│   │   ├── source_driver_ros1.hpp   # 修改：每帧写 timeshare
│   │   └── timeshare_writer.hpp     # 新增：mmap 写入器
│   └── driver/HesaiLidar_SDK_2.0/   # 子模块（禾赛 SDK 2.0）
└── TIMESHARE.md
```

## 2. timeshare 机制

每解码完一帧点云，驱动把该帧起始时间戳写入 `$HOME/timeshare`（mmap 共享文件），
供相机、IMU、触发节点读取并对齐。

- 单位：UTC 纳秒（uint64）
- 路径：`$HOME/timeshare`
- 布局兼容：
  - 旧版驱动只写 8 字节（offset 0）；
  - 本机相机/触发节点使用 `struct time_stamp { int64_t high; int64_t low; }`，读 offset 8；
  - 本实现创建 **16 字节**文件，**offset 0 与 offset 8 写入同一个纳秒值**，两种消费者都能读到。
- 代码位置：
  - `src/manager/timeshare_writer.hpp`
  - `src/manager/source_driver_ros1.hpp` 中的 `WriteTimeshare()` / `SendPointCloud()`

验证：

```bash
od -An -tu8 ~/timeshare
# 输出两列相同，单位为 ns
```

或：

```bash
python3 - <<'PY'
import mmap, os, struct
p = os.path.expanduser("~/timeshare")
with open(p, "rb") as f:
    mm = mmap.mmap(f.fileno(), 16, access=mmap.ACCESS_READ)
    print(struct.unpack("<QQ", mm[:16]))
PY
```

## 3. 编译（ROS1 Noetic）

```bash
mkdir -p ~/hesai_ws/src
cd ~/hesai_ws/src
git clone --recurse-submodules -b JT https://github.com/WilsonGuo/HesaiLidar_General_ROS_timeshare.git
cd ~/hesai_ws
source /opt/ros/noetic/setup.bash
catkin_make -j$(nproc)
```

依赖：`libboost-all-dev libyaml-cpp-dev libpcl-dev libpcap-dev libssl-dev
ros-noetic-pcl-ros ros-noetic-pcl-conversions`。

## 4. JT16 串口启动

`config/jt16_serial.yaml` 已按串口模式配置：

```yaml
source_type: 4
serial_type:
  rs485_com: /dev/ttyUSB0        # 点云
  rs232_com: /dev/ttyUSB1        # 命令/角度文件
  point_cloud_baudrate: 3000000  # 旧固件 00.B0.1/00.B0.5 用 3125000
  correction_save_path: ""
  correction_file_path: "path/to/JT16_Angle Correction File.csv"
```

启动前把 `correction_file_path` 改成实际角度文件路径（可从
`src/driver/HesaiLidar_SDK_2.0/correction/angle_correction/JT16_Angle Correction File.csv`
复制，或用 LidarUtilities 导出的本机文件）：

```bash
source /opt/ros/noetic/setup.bash
source ~/hesai_ws/devel/setup.bash
roslaunch hesai_ros_driver jt16_serial.launch
```

> `start.launch` 会启动 rviz，无显示环境请使用 `jt16_serial.launch`。

## 5. 型号支持

| 型号 | 状态 | 说明 |
|---|---|---|
| JT16 | ✅ | 串口（RS485 点云 + RS232 命令），本分支已适配 |
| JT128 | ✅ | 上游支持，走 UDP 1.4（`source_type: 1`） |
| JT32 | ❌ | 使用 UDP 1.12，当前公开禾赛 SDK/ROS 驱动无法解析 |

JT128 使用以太网 UDP 时，将 `config.yaml` 改为 `source_type: 1`，
配置 `device_ip_address`、`udp_port`、`correction_file_path`（JT128 CSV）即可；
本 timeshare 修改与型号无关，JT128 同样会自动写 timeshare。

## 6. 说明

- 旧版 `HesaiLidar_General_ROS` 代码保留在 `main` 分支，本分支为新的 SDK 2.0 驱动 + timeshare。
- SDK 子模块固定在 `357ff14`（v2.0.11-18）以保持与已验证构建一致；
  如需 JT128 最新修复可执行 `git -C src/driver/HesaiLidar_SDK_2.0 fetch && checkout origin/master` 后重编。
- timeshare 写的是 LiDAR 时钟；绝对时间仍需外部 PPS/GPRMC（或 PTP）同步。
- 代码遵循上游 BSD 许可。

# JT128 M8 接口线序说明

> 适用：禾赛 JT128（M8 接口）
> 依据：JT128 M8 接口线序图（2026-11-25，用户提供）
> 说明：本文件用于接线与和 GUNDA 同步器/ROS 驱动对接参考；最终以禾赛
> JT128 官方手册和实物丝印为准。

## 1. 雷达侧 CNN1（M8）

| 引脚 | 引线颜色 | 信号 | 线径 |
|---:|---|---|---|
| 6 | 红 | Power | 26 AWG |
| 1 | 黑 | GND | 26 AWG |
| 8 | 白 | GNSS PPS | 28 AWG |
| 7 | 绿 | GNSS NMEA | 28 AWG |
| 4 | 黄 | Ethernet TX+ | 28 AWG |
| 5 | 蓝 | Ethernet TX− | 28 AWG |
| 3 | 橙 | Ethernet RX+ | 28 AWG |
| 2 | 紫 | Ethernet RX− | 28 AWG |

## 2. 对外连接器侧

### 2.1 电源 CNN2

| 引脚 | 颜色 | 信号 | 说明 |
|---:|---|---|---|
| 1 | 蓝 | Power | **9 ~ 32 V DC** |
| 2 | 黑 | GND | 0 V |

### 2.2 GNSS CNN3

| 颜色 | 信号 | 说明 |
|---|---|---|
| 蓝 | GNSS PPS | **3.3 V**；上升沿 ≤ 500 ns；脉冲宽度建议 > 1 ms；周期 1 s（上升沿至上升沿） |
| 棕 | GNSS NMEA | **RS232 电平，−13 ~ 13 V** |
| — | GND | 外接 GNSS 模块地 |

支持的 NMEA 语句：**GPRMC / GNRMC**。

### 2.3 以太网 CNN4

| 引脚 | 颜色 | 信号 | 说明 |
|---:|---|---|---|
| 1 | 橙白 | Ethernet TX+ | 标准 100BASE-TX，差分 −1 ~ 1 V |
| 2 | 橙 | Ethernet TX− | 同上 |
| 3 | 蓝白 | Ethernet RX+ | 同上 |
| 6 | 蓝 | Ethernet RX− | 同上 |

## 3. 与 GUNDA 同步器接线

JT128 需要外部 GNSS 授时（PPS + NMEA），同步器提供：

| 同步器侧 | JT128 侧 | 说明 |
|---|---|---|
| LIDAR1 RMC TX（RS232） | GNSS NMEA（棕） | GUNDA 固件/硬件注释为 RS232；上电前实测空闲电平确认 |
| LiDAR PPS（3.3 V，1 Hz，100 ms 高） | GNSS PPS（蓝） | 满足 1 s 周期、脉宽 > 1 ms；上升沿 ≤ 500 ns 需示波器确认 |
| GND | GND | 共地 |
| — | Power（CNN2 1 脚） | **独立 9~32 V 供电**，不要从同步器取电 |
| — | Ethernet（CNN4） | 点云直连工控机/交换机，不经同步器 |

注意：

1. **JT128 出厂默认时钟源是 PTP**（1588v2 / 802.1AS）；PTP 模式不需要 PPS 线。
   使用 GUNDA 同步器的 PPS + GPRMC 前，必须先在 LidarUtilities（或 PTC API）里把
   **clock source 切换到 GNSS**，否则即使接好 PPS/NMEA 也仍然是 PTP/内部时间。
2. **JT128 的 NMEA 是 RS232 电平**，与 JT16 直连口的 3.3 V TTL 不同；
   如果同步器 LIDAR1 实测是 3.3 V TTL，需要加 RS232 电平转换（如 MAX3232）。
3. NMEA 波特率图中未标注；GUNDA LIDAR1 默认 9600，JT128 侧需按手册/实机配置确认，
   两边必须一致。
4. PPS 要求 3.3 V；GUNDA TIM2 输出为 3.3 V，周期 1 s、高电平 100 ms。
5. **GNSS 模式时序要求**（JT128 手册 J01）：PPS 周期 1 s ± 50 µs、脉宽 ≥1 ms
   （建议 10–100 ms）；NMEA 要在本秒 PPS 上升沿之后开始、在 PPS 下降沿之后结束，
   并且距下一秒上升沿至少 100 ms。
   GUNDA 的 GPRMC 约 72–85 ms 发完，而 PPS 高电平为 100 ms，NMEA 可能在
   PPS 下降沿之前就结束，**存在与 JT16 类似的时序偏差**：建议用示波器实测
   （或临时把固件 PPS 脉宽改成 10 ms、或给 RMC 增加约 40 ms 起始延时）后再确认锁定。
6. 供电范围 9~32 V，比 JT16（12~16 V）宽；建议按 JT128 手册留足电流余量并共地。

## 4. ROS 驱动配置（JT128）

JT128 点云走**以太网 UDP**，不是串口；`HesaiLidar_ROS_2.0` 支持 JT128（UDP 1.4，
自动识别）。示例：

```yaml
lidar:
  - driver:
      use_gpu: false
      source_type: 1                       # 1 = 实时雷达（UDP）
      lidar_udp_type:
        device_ip_address: 192.168.1.201   # 按实际雷达 IP
        udp_port: 2368
        ptc_port: 9347
        correction_file_path: "path/to/JT128_Angle Correction File.csv"
        firetimes_path: ""
      ros:
        ros_frame_id: hesai_lidar
        ros_send_point_cloud_topic: /lidar_points
        ros_send_imu_topic: /lidar_imu
        send_point_cloud_ros: true
        send_imu_ros: true
```

启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/hesai_ws/devel/setup.bash
roslaunch hesai_ros_driver start.launch
```

验证：

```bash
rostopic hz /lidar_points
rostopic echo -n1 /lidar_points/header
```

说明：

- JT128 的默认时钟源是 PTP；要使用 GUNDA 的 PPS + GPRMC，需要先在 LidarUtilities/PTC
  中把 clock source 切到 GNSS，并确认点云时间戳已经对到当前 UTC；
- JT128 的 timeshare 与 JT16 共用同一套实现：驱动启动后会自动写 `$HOME/timeshare`，
  不需要额外配置；
- JT128 的额外点字段（confidence / weightFactor / dirtyLevel / noiseLevel / envLight）
  当前 ROS 驱动固定使用 `LidarPointXYZIRT`，不会全部发布；
- JT128 的绝对时间来自 PPS + GPRMC，同步器锁定 GNSS 后点云时间戳才是当前 UTC。

## 5. 与 JT16 的接口差异

| 项目 | JT16 | JT128 |
|---|---|---|
| 电源 | 12 ~ 16 V | **9 ~ 32 V** |
| NMEA 电平 | 直连口 3.3 V（接线盒为 RS232） | **RS232（−13 ~ 13 V）** |
| PPS | 3.3 V CMOS | 3.3 V CMOS |
| 点云传输 | RS485 3 Mbps（或网口接线盒） | **以太网 100BASE-TX** |
| ROS 驱动接口 | `source_type: 4`（串口） | `source_type: 1`（UDP） |

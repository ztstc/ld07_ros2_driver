# LD07 LiDAR ROS 2 Driver

ROS 2 driver for the **LDROBOT LD07** solid-state LiDAR, written in C++ with no vendor SDK
dependency. Tested on **ROS 2 Jazzy** (Ubuntu 24.04).

LDROBOT LD07 固态激光雷达的 ROS 2 C++ 驱动，直接解析原生串口协议，不依赖厂商 SDK。

---

## 功能特性

- **纯协议实现**：直接解析 LD07 原生串口协议（921600 8N1），无需 LDRobot 官方 SDK
- **出厂矫正**：启动时读取雷达出厂的矫正系数（k/b）与每相机点数，并按厂商算法做畸变矫正
- **点云输出**：发布 `sensor_msgs/PointCloud2` 到 `/cloud`，160 点/帧，实测约 **28.8 Hz**
- **掉线自愈**：CH340 串口重新枚举（`ttyUSB0` → `ttyUSB1`）或供电跌落导致 USB 掉线后，自动重新发现端口、重新读取矫正参数并恢复扫描
- **启停服务**：通过 `std_srvs/Empty` 服务远程控制雷达开关
- **开箱即用**：附带 RViz2 配置与 launch 文件

---

## 硬件要求

| 项目 | 说明 |
| --- | --- |
| 雷达 | LDROBOT LD07 固态激光雷达 |
| 接口 | USB 转串口（雷达参考板上为 CH340） |
| 串口参数 | **921600** baud，8 数据位，无校验，1 停止位 |

> **供电提示**：LD07 激光器启动瞬间电流尖峰较大。若参考板使用能力不足的 LDO 供电，
> 会造成电压跌落 → CH340 复位 → USB 设备反复掉线。已验证需使用 **≥500 mA** 的 LDO。
> 详见 [故障排查](#故障排查)。

---

## 依赖

- ROS 2（Jazzy 已验证，其他较新版本应可直接使用）
- `rclcpp`、`sensor_msgs`、`std_srvs`（均为 ROS 2 基础包）

---

## 编译

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/ld07_ros2_driver.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 串口权限

当前用户需能访问 `/dev/ttyUSB*`：

```bash
sudo usermod -aG dialout $USER
# 注销并重新登录后生效
```

## 运行

只启动驱动节点：

```bash
ros2 launch ld07_ros2_driver start_ld07.launch.py
```

同时启动驱动与 RViz2（自动加载 `rviz/ld07.rviz`）：

```bash
ros2 launch ld07_ros2_driver start_ld07_rviz.launch.py
```

手动启停激光器（例如录制 rosbag 结束后让电机停转）：

```bash
ros2 service call /stop std_srvs/srv/Empty
ros2 service call /start std_srvs/srv/Empty
```

查看点云：`Fixed Frame` 设为 `ld07_frame`，添加 `PointCloud2` 话题 `/cloud`。

---

## 接口说明

### 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cloud` | `sensor_msgs/PointCloud2` | 点云，`frame_id = ld07_frame`，`width = 160`，`height = 1`，z 恒为 0 |
| `/scan` | `sensor_msgs/LaserScan` | 由 `enable_scan` 控制，**当前版本未发布**（源码中发布调用已注释，仅 point cloud 可用） |

> 无效测距点（距离为 0）会以 `(0, 0)` 占位输出，以保持点云索引与雷达通道一一对应。

### 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/start` | `std_srvs/Empty` | 打开激光、开始扫描 |
| `/stop` | `std_srvs/Empty` | 停止扫描、关闭激光 |

### 参数

配置文件：`config/params.yaml`（节点名 `ld07_node`）

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enable_scan` | `bool` | `true` | 是否创建 `/scan` 发布器 |
| `enable_cloud` | `bool` | `true` | 是否创建并发布 `/cloud` |
| `autostart` | `bool` | `true` | 节点启动后是否立即开始扫描 |

---

## 通信协议

雷达通过串口以固定帧格式通信：

```
AA AA AA AA | ADDR | CMD | OFFSET(2, LE) | LEN(2, LE) | DATA | CS
  0    1    2    3     4      5      6      7        8     9      ...   末字节
```

- **帧头**：`0xAA 0xAA 0xAA 0xAA`
- **校验和 CS**：从 `ADDR` 起累加到 `DATA` 最后一字节，取低 8 位

  $$\text{CS} = \left(\sum_{i=4}^{n-2} \text{byte}_i\right) \bmod 256$$

### 指令

| 指令 | 值 | 说明 |
| --- | --- | --- |
| 开始扫描 | `0x02` | 雷达持续推送距离帧 |
| 停止扫描 | `0x0F` | 停止推送并关闭激光 |
| 读取矫正参数 | `0x12` | 返回 18 字节矫正数据 |

### 距离帧（CMD `0x02`）

- `LEN = 0x144 = 324` 字节：4 字节时间戳 + 160 × `uint16`（小端）
- 数据区起始于帧内第 10 字节（即帧头 + ADDR/CMD/OFFSET/LEN 之后）
- 每个测量字：

```
位 0..8   → 距离 distance = value & 0x1FF   (mm)
位 9..15  → 置信度 confidence = value >> 9
```

### 矫正参数（CMD `0x12`，LEN = 18）

| 偏移 | 内容 |
| --- | --- |
| 0–3 | `k[0]`（`uint32` LE，÷10000） |
| 4–7 | `k[1]`（`uint32` LE，÷10000） |
| 8–11 | `b[0]`（`uint32` LE，÷10000） |
| 12–15 | `b[1]`（`uint32` LE，÷10000） |
| 16–17 | 每相机点数 `points_per_camera`（`uint16` LE，LD07 为 **80**） |

160 个测距点分为两组（各 80 点）共用两组系数；`angTransform()` 按厂商算法把
通道索引还原为角度与距离，再转换为笛卡尔坐标（单位：米）。

> 原始协议细节见 [`doc/LDROBOT_LD07_Development_Manual_v1.0_EN.pdf`](doc/LDROBOT_LD07_Development_Manual_v1.0_EN.pdf)。

---

## 目录结构

```
├── CMakeLists.txt
├── package.xml
├── config/params.yaml              # 节点参数
├── launch/
│   ├── start_ld07.launch.py        # 仅驱动
│   └── start_ld07_rviz.launch.py   # 驱动 + RViz2
├── include/ld07_ros2_driver/
│   ├── ld07_driver.hpp             # 串口协议与解析
│   └── ld07_node.hpp               # ROS 2 节点
├── src/
│   ├── ld07_driver.cpp
│   └── ld07_node.cpp
├── rviz/ld07.rviz
└── doc/                            # 厂商开发手册
```

---

## 故障排查

**点云不出 / USB 设备反复掉线**

典型序列：`ttyUSB0` 每隔数秒消失一次，`dmesg` 出现 `usb ... connect-debounce failed`。

- 原因：供电不足。发送 `0x02` 启动扫描后 0.1–0.2 s 内必掉线，而空闲（激光关闭）时稳定。
- 排查方法：确认空闲时不掉线、一启动就掉线即可定位。
- 解决：更换为 ≥500 mA 的 LDO，问题即可消除（实测可连续 60 s 以上无掉线、无重连）。

**`Failed to open port /dev/ttyUSB0`**

驱动会在打开时自动在 `/dev/ttyUSB*` 中重新发现端口，通常无需干预。请确认设备存在
（`ls /dev/ttyUSB*`）且当前用户属于 `dialout` 组。

**主机端读取程序导致帧错乱**

用 Python 等脚本以「读一段 → 睡一会」的方式读取 921600 baud 数据流会撑爆内核 tty
缓冲区并破坏帧边界。请使用紧凑的读取线程，或直接使用本驱动节点。

---

## 许可证

[Apache-2.0](LICENSE)

原始实现作者：Bram Fenijn &lt;bafenijn@gmail.com&gt;



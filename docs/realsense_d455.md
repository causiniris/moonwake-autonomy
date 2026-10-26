# RealSense D455 相机接入

本仓库在 `src/realsense2_camera` 与 `src/realsense2_camera_msgs` 中纳入了 [RealSense 官方 ROS 2 驱动](https://github.com/realsenseai/realsense-ros)的固定版本。上游版本、许可与源码范围见 [上游源码记录](../third_party/realsense_ros/README.md)。

## 环境与依赖

- Ubuntu 22.04、ROS 2 Humble；
- D455 相机与可用的 USB 3 连接；
- RealSense SDK `librealsense2`，版本至少 2.56，并提供开发文件。SDK 2.56.5 是与本次驱动版本相近、包含 D455 修复的官方发布版；实际固件与 SDK 组合需在目标设备上确认。

按 [RealSense 官方 Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)选择一种 SDK 安装方式。若通过官方 Debian 仓库安装，应包含 `librealsense2-dev`；内核模块安装步骤以官方说明和目标主机内核为准。避免同时安装多个来源的 SDK。

## 构建

从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -r -y
colcon build --symlink-install --packages-up-to realsense2_camera
source install/setup.bash
```

先运行 `rs-enumerate-devices`，记录相机型号、序列号、固件版本、连接方式及支持的彩色和深度流配置。设备实际支持的配置应作为 Launch 参数选择依据。

## 最小启动与检查

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=d455
```

再开一个已加载 ROS 2 与本仓库 `install/setup.bash` 的终端，检查：

```bash
ros2 topic list
ros2 topic hz /camera/d455/color/image_raw
ros2 topic hz /camera/d455/depth/image_rect_raw
ros2 topic echo /camera/d455/color/camera_info --once
```

官方驱动也发布相机内部的 TF 与传感器外参。整船接入时还需测量并配置船体 `base_link` 到相机安装位的变换，确认 TF 由单一节点发布。需要固定某台相机时，使用 `serial_no`；官方 Launch 要求纯数字序列号前加 `_`，例如 `serial_no:=_123456789012`。

识别节点只需要彩色图时，可设置 `enable_depth:=false`，并保持深度对齐、点云与 IMU 关闭。识别需要对应像素的深度时，再评估 `enable_sync:=true` 和 `align_depth.enable:=true`，并检查图像时间戳、坐标系、QoS 与实际帧率。最终参数应保存在本仓库的启动配置中。

当前源码导入尚未代表已完成设备验证。合入前应在目标 Ubuntu 主机完成构建、连续采集、断线重连、岸上日光与水面场景验证，并记录实际结果。

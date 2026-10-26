# RealSense ROS 2 上游源码记录

本仓库将 RealSense 官方 ROS 2 驱动的必要源码直接纳入 `src/`，使相机节点与接口定义随本仓库版本一起构建、Review 和部署。

- 上游仓库：https://github.com/realsenseai/realsense-ros
- 上游发布标签：`4.56.4`
- 上游 Commit：`bafc21080c5c8e259dadbb309797949aee0dd950`
- 纳入的 ROS 2 Package：`src/realsense2_camera`、`src/realsense2_camera_msgs`
- 上游许可与声明：本目录中的 `LICENSE`、`NOTICE.md` 和 `Intel Copyright`

上述两个 Package 保持上游发布版内容不变。上游的 `realsense2_description` 是可选的相机模型包，含体积较大的网格文件，当前采集链路未使用，因此没有纳入；相机驱动自带的部分点云示例 Launch 依赖该可选包。

`librealsense2` 是 `realsense2_camera` 的系统级构建与运行依赖，未纳入本仓库。此发布版的 CMake 要求 SDK 版本至少为 2.56。安装与构建说明见 [D455 接入说明](../../docs/realsense_d455.md)。

更新上游源码时，先选定发布标签并记录 Commit，再同步这两个 Package 与许可声明；检查上游依赖或文件布局是否变化，并在 Ubuntu 22.04 + ROS 2 Humble 上重新构建和验证相机。

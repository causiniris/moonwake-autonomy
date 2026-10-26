# Moonwake Autonomy

`moonwake-autonomy` 是 Moonwake 战队无人船的上位机自主系统总仓库，用于集中维护相机采集、视觉感知、定位与多传感器融合、规划导航、上位机与 MCU 通信桥接，以及整船启动配置。

本仓库采用 **ROS 2 Monorepo**：一个 Git 仓库对应一个 ROS 2 Workspace，所有上位机 Package 在同一个版本下构建、联调和部署。MCU 固件不放在本仓库中，但双方使用的接口和协议必须具有明确、可追踪的版本。

> 当前状态：相机任务分支已纳入 RealSense 官方 ROS 2 驱动的两个必要 Package；尚待在目标 Ubuntu 主机完成 SDK 安装、构建与 D455 实机验证。

## 开发环境

- Ubuntu 22.04
- ROS 2 Humble
- C++17
- Python 3（主要用于 Launch 和工具脚本）
- CMake、`ament_cmake` 与 `colcon`
- Git 与 GitHub Fork 工作流

运行时代码以 C++ 为主。开发环境以 Ubuntu 22.04 为准；Windows 主要用于仓库管理和文档维护，不作为 ROS 2 运行环境。

## 仓库结构

```text
moonwake-autonomy/
├── src/                         # 所有 ROS 2 Package
├── build/                       # colcon 生成，不提交
├── install/                     # colcon 生成，不提交
├── log/                         # colcon 生成，不提交
├── .github/
│   └── pull_request_template.md
├── docs/                        # 本仓库的设备接入与验证说明
├── third_party/                 # 纳入的上游源码版本与许可声明
├── .editorconfig
├── .gitattributes
├── .gitignore
├── CONTRIBUTING.md
└── README.md
```

仓库根目录只有一套 `build/`、`install/` 和 `log/`。每个 Package 根据实际需要维护自己的 `src/`、`include/`、`launch/`、`config/` 和 `test/`，不得在 Package 内部重复创建工作区级构建目录。

已纳入的第三方 ROS 2 Package 如下。来源、版本和许可见 [RealSense 上游源码记录](third_party/realsense_ros/README.md)。

| Package | 职责 |
| --- | --- |
| `realsense2_camera` | D455 相机采集与 ROS 2 话题发布 |
| `realsense2_camera_msgs` | 相机驱动使用的消息、服务与 Action |

计划中的自有模块边界如下。表中的 Package 会在对应功能确实需要本仓库代码时创建，不提前建立没有内容的空包。

| Package | 职责 |
| --- | --- |
| `moonwake_interfaces` | 自定义 Message、Service、Action 与接口兼容约定 |
| `moonwake_camera` | 官方驱动不能覆盖时的相机适配代码；启动参数可先放在 `moonwake_bringup` |
| `moonwake_perception` | 目标检测、跟踪与视觉测量 |
| `moonwake_localization` | 定位观测与位姿估计 |
| `moonwake_fusion` | 当融合实现需要独立维护时，承载 GNSS、IMU、视觉等多传感器状态估计 |
| `moonwake_planning` | 任务与路径规划 |
| `moonwake_navigation` | 导航决策与局部目标生成 |
| `moonwake_mcu_bridge` | 上位机与 MCU 之间的通信桥接 |
| `moonwake_bringup` | 整船 Launch、参数组合与统一启动入口 |

## 获取代码

正式开发采用 Fork 工作流。请先在 GitHub 上 Fork 本仓库，再 Clone 自己的 Fork：

```bash
git clone git@github.com:<你的GitHub用户名>/moonwake-autonomy.git
cd moonwake-autonomy

git remote add upstream git@github.com:moonwake-vision/moonwake-autonomy.git
git remote -v
```

固定约定为：

```text
origin    = 个人 GitHub Fork
upstream  = moonwake-vision/moonwake-autonomy
```

开始新任务前，从最新的正式 `main` 创建短期任务分支：

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
git switch -c feat/camera-usb-capture
```

不要直接在个人 Fork 的 `main` 上开发。完整的分支、Commit、PR 与 Review 规则见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 构建与测试

相机驱动源码依赖单独安装的 RealSense SDK `librealsense2`，最低版本为 2.56。请先按 [D455 接入说明](docs/realsense_d455.md)安装并确认 SDK，再在 Ubuntu 22.04 中从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src --skip-keys=librealsense2 -r -y
colcon build --symlink-install

source install/setup.bash
colcon test
colcon test-result --verbose
```

修改 C++ 源码后必须重新构建。不要把 `--symlink-install` 理解为 C++ 代码无需编译。

相机的启动方式、参数与验证方法见 [D455 接入说明](docs/realsense_d455.md)。其他 Package 的具体说明由各自 README 维护。

## 版本与验证含义

- 任务分支：正在开发或等待 Review；
- `main`：已经完成 Review、可以构建，并通过当前阶段约定的岸上验证；
- `onboard-YYYYMMDD-NN` Tag：已经记录一次指定的船上或下水验证；
- `release/*`：仅在比赛版本需要冻结，而 `main` 仍需继续开发时由 Maintainer 创建。

岸上编译成功不代表代码已经安全通过实船验证。与推进器、舵机或安全链有关的输出，在完成接口评审、岸上联调和规定的实船验证前，不得作为真实控制指令使用。

## 数据、模型与秘密信息

下列内容不得直接提交到普通 Git 历史：

- `build/`、`install/`、`log/` 和其他生成物；
- 数据集、模型权重、TensorRT Engine、rosbag、原始视频和大体积日志；
- 密码、Token、SSH 私钥、设备密钥和真实 `.env`；
- 个人绝对路径和只适用于单台电脑的临时配置。

ROS 2 参数、传感器标定和正式运行配置通常是可复现性的一部分，不应被笼统排除。含秘密或本地信息的配置应提交脱敏的 `.example` 模板，并忽略真实文件。

## 参与协作

所有修改通过任务分支和 Pull Request 进入 `main`。PR 必须说明修改背景、主要改动、实际验证、影响范围和已知限制。接口、依赖、参数、启动方式或部署步骤发生变化时，应在同一 PR 更新对应文档。

问题、设计讨论和无法复现的异常优先通过 GitHub Issue 记录，避免重要结论只保留在个人聊天记录中。

## License

本仓库自有代码目前尚未选择开源许可证。在正式添加根目录 `LICENSE` 前，请勿假定自有内容已经获得对外复制、修改或再发布授权。纳入的 RealSense 上游源码依照其独立的 Apache 2.0 许可与声明分发，见 [third_party/realsense_ros](third_party/realsense_ros/README.md)。

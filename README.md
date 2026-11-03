# ACG138 FPGA + RK3588 机器人

面向 ACG138 平台的双轮机器人源码：FPGA 承担传感器解析、运动控制与保护，RK3588 承担 OV5647 图像采集、人员/瓶子识别、任务请求、HDMI 显示和录像。

**当前工程记录中的板上固件为雷达地图观察版 `0000F7C8`，雷达运动接管关闭。** revision18 接管源码已合入，但物理参数标定和实车接管尚未验收。详细状态见 [STATUS](docs/STATUS.md)。

## 已实现的功能

- FPGA 轮速 PI/PWM、编码器与 IMU 反馈、遥控手动运动、停车保护、扫描绕障和路径点导航。
- RK 通过 CSI0/FPC2 采集 OV5647，使用 OpenCV DNN 在 CPU 上运行 YOLOv8 ONNX，保留 COCO 的人和瓶子两类结果。
- 人员跟随、沿路径搜索瓶子，以及基于预设地点的中文任务输入。
- X3 Pro 雷达原始字节经 RK USB 转发至 FPGA；FPGA 完成包解析、前/左/右测距、36 扇区地图及规划诊断。
- HDMI 摄像头页、独立雷达地图页、中文状态叠加，以及 U 盘分段录像。

这里描述的是源码功能。地面导航精度、完整带动力跟随、真实转角和雷达接管仍需实车验证。识别当前未使用 RKNN/NPU；“瓶子”分类不判断容器内容物。保留的 FPGA DVP 取像代码也不代表 FPGA 直采已经验收。

## 从这里开始

1. 阅读 [接线说明](docs/WIRING.md)，核对板卡、电源、端口和电平。
2. 按 [构建与部署](docs/BUILD.md)准备工具链、Python 依赖、模型及板端配置。
3. 按 [操作说明](docs/OPERATIONS.md)检查画面和遥测，再进行对应模式的操作。
4. 复现或修改控制逻辑前，阅读 [系统架构](docs/ARCHITECTURE.md)与 [验证状态](docs/STATUS.md)。

上传步骤见 [UPLOAD](docs/UPLOAD.md)，本次发布副本的离线检查见 [VALIDATION](docs/VALIDATION.md)。

## 目录与入口

| 路径 | 用途 |
| --- | --- |
| [`robot_rebuild/fpga`](robot_rebuild/fpga) | Gowin 工程、约束、自编 RTL 和依赖 RTL |
| [`robot_rebuild/rk`](robot_rebuild/rk) | 相机、识别、串口协议、跟随、导航、任务与录像模块 |
| [`ch341_driver_work/hdmi_camera.py`](ch341_driver_work/hdmi_camera.py) | 实际使用的 GTK HDMI 界面 |
| [`ch341_driver_work/sync_hdmi.py`](ch341_driver_work/sync_hdmi.py) | 板端文件备份、校验与部署 |
| [`robot_rebuild/tools`](robot_rebuild/tools) | 模型导出、雷达候选表生成、运动模型及离线遥测工具 |
| [`robot_rebuild/tests`](robot_rebuild/tests) | Python 回归和 RTL 仿真测试 |
| [`docs`](docs) | 架构、构建、接线、操作和状态 |

`robot_rebuild/rk/main.py` 是独立检查程序，键盘控制方式不同于 GTK 界面。`browser_viewer.py` 是只读浏览器监视程序。这些程序不应同时占用同一个 FPGA 串口。

## 常用操作

| 操作 | 行为 |
| --- | --- |
| 遥控 `0` | 在人员跟随模式中明确启动跟随 |
| 遥控 `5` | 停车；条件恢复后清除相关锁定状态 |
| 遥控 `7` | 直接请求沿配置路线逐点搜索瓶子，无需再按 `0` |
| 遥控 `9` | 开始/停止 U 盘摄像头录像 |
| `2` / `8`、`4` / `6`、`1` / `3` | 前进/后退、前向左/右转、定点左/右旋 |
| `VOL±` | 调整当前模式的速度档 |

方向键点按后持续执行，松手不会自动停车。完整流程、任务六方向和地图含义见 [操作说明](docs/OPERATIONS.md)。

## 使用范围与第三方内容

本包用于源码阅读、构建、离线验证和后续硬件调试。历史部署状态不等于从本包新构建的产物已验收，也不保证换板后直接运行。模型、厂商工具与相关依赖的来源和授权边界见 [THIRD_PARTY](THIRD_PARTY.md)。

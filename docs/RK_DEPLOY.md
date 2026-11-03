# RK3588 部署

这份源码沿用 ACG138 的 Ubuntu 22.04 板卡系统与 `ubuntu` 用户。它不是通用 RK3588 系统镜像。相机设备树、内核驱动及 ISP/AIQ 库由匹配的板卡 BSP 提供，不在本仓库内。

## 依赖和识别模型

在 RK 上安装系统 Python 的运行依赖：

```bash
sudo apt install python3-numpy python3-opencv python3-serial python3-gi python3-gi-cairo gir1.2-gtk-3.0 python3-cairo python3-pil fonts-noto-cjk v4l-utils ffmpeg x11-xserver-utils
mkdir -p /home/ubuntu/fpga-hdmi-viewer /home/ubuntu/.config/autostart
```

在开发电脑的独立 Python 环境中导出识别模型：

```bash
python -m venv .venv-model
# 激活该环境后执行
python -m pip install ultralytics==8.3.0 onnx==1.16.2
python robot_rebuild/tools/export_model.py
```

脚本使用 YOLOv8n COCO 权重，输出 `robot_rebuild/rk/models/yolov8n.onnx`，输入为 320×320。首次取得权重需要网络；已有权重可放到同目录的 `yolov8n.pt`。模型文件不随源码包发布。模型及导出工具的使用条件见 [第三方来源](../THIRD_PARTY.md)，原模型记录见 [model_manifest.json](model_manifest.json)。环境不同重新导出后，哈希不保证与历史记录相同。

板上使用 OpenCV CPU 推理，只保留 person（0）和 bottle（39），没有部署 RKNN/NPU 推理。

## 从开发电脑同步

先在终端用普通 SSH 连接板子，核对主机指纹并写入本机 `known_hosts`，不要跳过主机身份核验。部署脚本优先使用 SSH 密钥/agent，也支持运行时输入密码。仓库内不保存密码。

```powershell
python -m pip install -r requirements-dev.txt
$env:ROBOT_HOST = '169.254.100.100'  # 替换成实际板端地址
$env:ROBOT_USER = 'ubuntu'
# 可选：$env:ROBOT_SSH_KEY = '本机私钥路径'
python ch341_driver_work/sync_hdmi.py --mission robot_rebuild/rk/mission_semantic_example.json
```

同步脚本把 GUI、算法模块、模型和配置平铺到 `/home/ubuntu/fpga-hdmi-viewer`，先备份并校验上传内容。更新时保留板上已有的地点、路径、录像和雷达配置。首次部署前先创建上述目录并准备模型；不要只复制 `hdmi_camera.py`。同步会更新并重启界面，操作前停车并断开电机动力。

生产入口是 `ch341_driver_work/hdmi_camera.py`。`robot_rebuild/rk/main.py` 是独立检查入口，两者不可混同。

## OV5647 与启动

当前配置为 OV5647、CSI0/FPC2、1296×972、RAW 后端，画面旋转 180°。按实际安装调整 `camera_config.json`。只在断电时插拔排线。

把 `robot-ov5647.service` 安装到板上的 `/etc/systemd/system/` 后，按需启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-ov5647.service
```

该服务通过 `prepare_ov5647.py --serve --index 0` 配置相机。当前 RAW 后端不依赖额外的 AIQ 可执行文件；选择 ISP 后端时需匹配 BSP 的 AIQ 程序、IQ 文件及库，不能凭本源码包替代系统适配。

HDMI 界面依赖已登录的图形桌面。`restore_hdmi_display.sh` 和对应 `.desktop` 文件用于桌面显示恢复，安装位置按 `.desktop` 中的 `Exec` 配置。

## U 盘录像

默认 `recording_config.json` 的 `usb_mount` 为空，由程序查找已挂载的 USB 存储。按 9 开始，再按 9 停止并保存。录像会叠加状态数据，先看到保存完成再拔盘。

若要为一只 FAT32 U 盘配置固定自动挂载，先用 `lsblk -f` 查其真实 UUID。在 RK 上把 `install_usb_automount.py` 和 `robot-usb.mount.in` 放在一起，再执行：

```bash
sudo python3 install_usb_automount.py --uuid 实际UUID
```

安装脚本不格式化磁盘，仅为指定 USB 分区建立挂载单元，要求 `ubuntu` 的 UID/GID 均为 1000。执行前停止录像。它与已挂载到其他位置的分区发生冲突时会拒绝安装，需要先正常卸载。

## 测试范围

无板卡单元测试不需要 GPIO、相机或网络。模型实际推理测试需另运行 `robot_rebuild/tests/test_model.py`，并按该脚本准备 Ultralytics 的 `bus.jpg`。实车转角、轮距、制动距离、雷达安装角度和接管行驶仍需现场标定验证。

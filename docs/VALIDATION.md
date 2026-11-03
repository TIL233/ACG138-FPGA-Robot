# 本次发布副本验证

验证日期：2026-11-03。以下结果在本上传副本上重新执行，区别于 [STATUS](STATUS.md) 中的历史板端记录。

| 检查 | 结果 |
|---|---|
| ModelSim 默认源码回归 | 81 个 `tb_*.sv` 全部通过，各日志包含 PASS，无 Fatal/Error |
| Python unittest discovery | 448 项通过，无跳过；已使用本次 RTL 生成的两份快照进行联测 |
| SSH 配置模拟测试 | 10 项通过，覆盖密钥优先、密码回退、主机密钥拒绝、端口和异常清理 |
| 观察构建准备 | `observe` / `observe_map` 各 45 个输入文件；地图配置只在副本更改显示开关 |
| 工程引用 | 42 个 Gowin 工程文件引用全部存在 |
| 发布清单 | 检查 Python 语法、JSON、文档链接、源码来源哈希、敏感文件和压缩包完整性 |

本轮使用已有的 NumPy 1.26.4、OpenCV 4.10.0、PySerial 3.5 及 Paramiko 5.0.0 依赖；这些第三方安装目录不随包发布。首次 Python 测试受受限临时目录权限影响，改用可写工作区临时目录后完整重跑通过，没有为了消除错误修改控制或算法代码。

本轮只做本机离线检查，没有连接、同步、烧录或启动小车，也没有重新运行 Gowin 综合和布局布线。真实模型推理、GTK 现场显示、实际 U 盘自动挂载及接管行驶未在本轮验收。录像测试中的损坏 JPEG 用例可能产生解码警告，以测试最终结果为准。

原工作工程保持不变。上传副本保留 FPGA RTL、串口协议和运行算法的原文件，只调整以下已有辅助文件以便换电脑使用：

- `ch341_driver_work/board_access.py`：用运行时环境变量、SSH 密钥/agent 或交互密码替代私人密码文件。
- `robot_rebuild/tests/run_rtl.ps1`：ModelSim 路径改为参数、环境变量或 PATH 查找。
- `robot_rebuild/rk/install_usb_automount.py` 与 `robot-usb.mount.in`：U 盘 UUID 改由安装时明确指定。

来源记录已去除个人电脑路径。新加的构建入口只提供观察与地图观察配置，不自动开启接管。文件清单见 [SOURCE_MANIFEST.json](../SOURCE_MANIFEST.json)：哈希统一按 LF 换行并去除 UTF-8 BOM 计算，保留原文件编码（含第三方旧编码注释），避免 Git 转换行尾造成误报。

复现入口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File robot_rebuild/tests/run_rtl.ps1 -SimulatorBin '你的ModelSim目录'
python -m unittest discover -s robot_rebuild/tests -p 'test_*.py'
python -B scripts/test_board_access.py
python scripts/build_fpga.py observe_map --prepare-only
```

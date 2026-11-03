# 构建与本地验证

以下命令在仓库根目录运行，保留 `robot_rebuild` 和 `ch341_driver_work` 两目录的并列关系。它们仅运行本地构建或测试，不连接开发板。板端依赖与部署见 [RK_DEPLOY.md](RK_DEPLOY.md)。

## FPGA 构建

器件是 Gowin GW5AT-60B，料号 `GW5AT-LV60PG484AC1/I0`。需要自行安装 Gowin IDE 及匹配许可证；历史构建使用 V1.9.12.04。Python 建议 3.10 或更新版本。构建前按接口文档确认 P20 跳线、红外接收电平及 `robot.cst`；脚本不检测实体接线。

```powershell
python scripts/build_fpga.py observe --prepare-only
python scripts/build_fpga.py observe_map --prepare-only
```

`--prepare-only` 不需要 Gowin；每次在 `build/<profile>_<timestamp>/fpga` 生成独立输入副本，保存源文件和副本的 SHA256。可指定 `--attempt my_run`；已有目录会被拒绝，不覆盖历史结果。

| 配置 | LIDAR_MAP_DISPLAY | 雷达运动接管 | 用途 |
| --- | --- | --- | --- |
| `observe` | 0 | 关闭 | 主源码默认观察配置 |
| `observe_map` | 1 | 关闭 | 增加地图遥测的观察配置 |

两种配置均保持 `LIDAR_PLANNER_ENABLE=1`，`LIDAR_CONTROL=0`、`LIDAR_ADVANCED_CONTROL=0`、`LIDAR_MODEL_VALIDATED=0`。地图配置只在构建副本改变 `LIDAR_MAP_DISPLAY`，不改仓库 RTL。F7C8 历史固件使用此地图观察参数；新构建不承诺产生相同 UserCode 或逐字节相同的位流。名义接管配置只曾用于综合验证，物理模型尚未标定，本发布入口不提供该配置。

实际构建可显式提供命令行工具：

```powershell
python scripts/build_fpga.py observe_map --gowin 'C:\Gowin\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe'
```

工具查找顺序是 `--gowin`、`GOWIN_SH` 环境变量、`PATH` 上的 `gw_sh`。`GOWIN_SH` 应填写可执行文件路径，不附带命令行参数。例如：

```powershell
$env:GOWIN_SH = 'C:\Gowin\Gowin_V1.9.12.04_x64\IDE\bin\gw_sh.exe'
python scripts/build_fpga.py observe
```

输出目录包含 `sources.json`、`status.json`、`build.log`；成功生成的位流通常在 `fpga/impl/pnr/robot.fs`。此入口沿用主工程 `build.tcl` 的 `place_option 2`，不放宽原时钟约束。`tools/project.py` 会重生成工程和引脚文件，普通构建不需要运行它。

Gowin 退出码为 0 仅表示工具完成。检查副本内 `fpga/impl/pnr/robot_tr_content.html` 的全部 setup/hold 违例、TNS、时钟约束与 Fmax；全部分析时钟须满足约束。主要约束为 sys50 20 ns、camera_pclk 10 ns、PLL CLKOUT0 20 ns、CLKOUT1 8 ns、CLKOUT2 41.333 ns。还要核对 `status.json` 的输入未改变标志及实际位流哈希。脚本不自动授予时序签核，构建或仿真通过也不代表带动力运动、标定或冷启动通过。

## RTL 与 Python 回归

先安装 Python 依赖，使用 ModelSim/兼容 Questa 的 `vlib`、`vlog`、`vsim`：

```powershell
python -m pip install -r requirements-dev.txt
powershell -NoProfile -ExecutionPolicy Bypass -File robot_rebuild/tests/run_rtl.ps1 -SimulatorBin 'C:\win64'
python -m unittest discover -s robot_rebuild/tests -p 'test_*.py'
```

`-SimulatorBin` 是工具所在目录，优先于 `MODELSIM_BIN` 环境变量；两者都未提供时从 `PATH` 查找三项工具。脚本在编译前检查工具是否齐全，仍逐个执行所有顶层 `tb_*.sv`，要求进程成功、日志没有 Fatal/Error 且含 PASS。仿真库和日志写入 `robot_rebuild/tests`，属于生成物。

```powershell
$env:MODELSIM_BIN = 'C:\win64'
powershell -NoProfile -ExecutionPolicy Bypass -File robot_rebuild/tests/run_rtl.ps1
```

应先 RTL 后 Python：`tb_video` 生成 `ethernet.hex`，`tb_lidar_advanced_top` 生成 `planning_diagnostics.hex`，供两项跨 RTL/Python 联测使用。没有这些文件时相应 Python 测试跳过；报告中应保留跳过数。主 RTL 回归使用仓库默认参数，不能代替新地图配置的完整时序与板端验收。

`robot_rebuild/tests/test_model.py` 是单独执行的真实 ONNX/OpenCV 烟雾测试，其 `main()` 不会被 unittest discovery 调用。模型获取、CPU 导出与其测试资源准备见 [RK_DEPLOY.md](RK_DEPLOY.md)。一般 Python 回归中的合成检测测试不能替代真实模型识别验收。

# 第三方来源说明

本发布副本保留了源码中的原作者署名、版权声明与来源记录，不附适用于整个仓库的统一开源许可证。保留署名和来源本身不代表已取得额外的再分发授权；第三方代码的适用条款仍以原权利人提供的许可为准。

| 材料 | 本地可核对的来源或声明 |
| --- | --- |
| `robot_rebuild/fpga/vendor/` 中的板卡参考 RTL | 来源记录见 `robot_rebuild/docs/source_manifest.json`；部分文件保留芯路恒/Corecourse 署名，未另附统一再分发许可。 |
| `robot_rebuild/fpga/vendor/gowin_pll_mod.v` | 文件头保留 Gowin 版权及 `All rights reserved` 声明，属于生成的厂商 IP 文件。 |
| `robot_rebuild/fpga/vendor/crc32_d8.v` | 原 Easics 文件头注明：使用和分发时保留版权声明，派生作品保留原声明和免责声明。 |
| YOLOv8n 导出流程 | `robot_rebuild/tools/export_model.py` 记录 Ultralytics 来源及其许可提示。模型权重和导出模型不随本副本分发，使用者自行取得并核对上游条款。 |
| Python 运行依赖 | NumPy、OpenCV、PySerial、Paramiko 等由使用者按依赖说明单独安装，依其各自许可使用。 |

本副本不携带下载模型、第三方 Python 依赖目录、轮子包、Linux/BSP 完整源码归档、开发工具链、许可证文件或本机编译缓存。Gowin/ModelSim 等工具需另行安装和授权。第三方原始源码里的版权注释不能因整理发布而删除。

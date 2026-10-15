# Snapshot 使用说明

这是2026-10-15当前remote5g_ws的事实与证据快照。已核实、源码、历史、待确认状态以00和verification_summary.json为准。前置30fps/目标50Hz为后续授权修订，见frequency_update_20261015.json；runtime中旧图快照仍保留，不重写历史记录。

- key_facts.json：关键事实与真实运动/回放结果。
- packages.json：2个源码包与依赖；project_tree.txt：精简工程树。
- topics.csv、ros_graph.json、tf_tree.json：当前运行图、话题/可靠性、TF边。
- vehicle_default.urdf / vehicle_competition.urdf及inventory.json：同一模型两个相机尺寸配置，非新建车辆。
- resources.json：803条资源路径/引用/大小，含派生副本；不复制大型DAE/纹理。
- source/：43份小文本源码/配置副本，原相对目录保留，已刷新授权变更文件；不是可单独运行的完整源码仓库。
- evidence/：本地命令、帮助、退出码和日志；packages_before_snapshot_ignore.txt是修复文档副本重复发现前的历史，不是最终包清单。
- runtime/competition/runtime.json与flat/runtime.json：实际数据摘要，图像只存PNG，不展开百万字节原始数组。
- runtime/competition/sample_bag：本地303KiB样例，metadata和info可读，db3留本地；静止样例不包含图像与控制运动。
- runtime/competition/screenshots：gzclient原生截图，可选择性分享；原始相机PNG在对应runtime目录。
- rules_excerpt.txt、rules_metadata.json：本地规则节选与hash，不声称当届最新。
- original_file_hashes.json：初始导出986个既有文件hash；当前允许修订清单6项变化，其余复核；不是整个home目录清单。
- collect_*.py、probe_runtime.py、verify_runtime.py、verify_playback.py：本次审计辅助；verify_runtime会启动专用仿真并在平坦场发小指令。未经理解不要重跑，不能用于在线实车。
- write_documents.py、finalize_handoff.py：仅生成本交接文档和轻量导出，不是比赛算法。

不含SSH私钥、Token、个人完整环境、真实支付/5G凭据。硬件PWM说明不复制。COLCON_IGNORE隔离整个交接目录，必须保留。

轻量text.zip包含12份Markdown、合并文件和关键文本/源码，不含bag数据库、大资源或运行时派生模型。ChatGPT若需要进一步核对，只上传所需小文件；不能因为看到本机路径就假定能直接读取用户电脑。

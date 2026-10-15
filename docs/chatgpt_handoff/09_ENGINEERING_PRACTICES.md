# 09 软件工程与实验管理现状

配置已于2026-10-15更新：前置30fps，速度/转角目标及反馈50Hz，drive.py定时50Hz，RSP上限50Hz；云台仍15fps。当前实测见 [频率修订报告](FREQUENCY_UPDATE_20261015.md)。未标修订的原始运行日志属于此前20Hz/15fps调查，不能当当前频率证据。

初始任务只调查；本次用户另行授权更新30fps/50Hz配置、指令定时发布与TF上限，未重构包或重装依赖；当前上传任务另获用户授权，已初始化Git/main并整理忽略规则。

## 现有基础

| 项目 | 当前事实 | 如何使用/边界 |
|---|---|---|
| Git | [V] 当前工作区Git仓库分支main；已推送至GitHub私有仓库JamieK32/remote5g_ws | 用git status、git log、git remote -v查看状态；远端地址及可见性验证见snapshot/repository_setup.json |
| 源码分层 | [S] sim与competition两包分离，插件/模型/场景/测试分目录 | 仿真复用sim，不复制车辆代码；视觉/任务包尚未创建 |
| 配置 | [S] vehicle.yaml、competition.yaml、parameter_sources.yaml、派生resolved.yaml | 机械和任务场景参数分开；来源性质已标注，未实测不得改称实测 |
| 依赖 | [V/S] package.xml/CMake明确ROS依赖；QR脚本固定缓存包版本 | 部分依赖在工程外用户缓存，缺乏完整机器可复现环境锁；本次不安装 |
| 构建/启动 | [V/S] colcon/ament，已有install；两个launch，轨道独立脚本 | 修改源码后才编译；单开轨道不等于插车 |
| 日志 | [V/S] colcon log、launch控制台、diagnostics、rosout、reports日志 | 当前可用于排障，缺统一实验索引/保留策略 |
| 单元测试 | [S/H] control_math_test.cpp；test_geometry.py；model_check.py | 历史有成功证据，本次没有重跑colcon test |
| 仿真集成 | [S/H] scripts/test_xtnetrc.sh、regression/smoke、run_matrix、integration/CLI | 会发控制/建删夹具，只在独立仿真运行；不盲目在用户在线场景启动 |
| 本次小验证 | [V] 两场景启动、真实接口、低速Ack/键盘、短bag录放、GUI启动 | 不是完整比赛回归，更不是自主算法成功 |
| 录放数据 | [N/S] 原data基本预留；本次新增303KiB审计bag和方法 | 尚无按任务/版本组织的视觉数据集 |
| 参数版本 | [S] source分类、resolved YAML、随机seed和坐标输出 | 原调查以文件hash留证；当前Git仓库建立后，新实验再记录对应commit |
| 结果管理 | [S/H] reports/xtnetrc、reports/competition含成功/失败历史 | 应读本次/指定修正版，不能只挑PASS或混入旧布局结果 |
| 性能/验收 | [S/H] 物理/几何/接触/四轮停车及相机检查 | 尚无白线误差、避障完成率、端到端任务性能基线 |
| CI/分支流程 | [V/N] 已有本地main；未建立CI或正式协作分支约定 | 先本地小测试，不第一天引入复杂流水线 |

## 最小改进顺序（建议，未执行）

1. 从下一次小实验开始，在data/experiments下用“日期_阶段_编号”建记录；留一张图、短日志/短bag和结论即可。
2. C阶段编码前，导师先解释Git工作区/暂存/commit，在现有main仓库练习git status/diff/log并审阅.gitignore，不重复初始化。别把data大bag、build/install/log、密钥和海量生成资源全提交。
3. 第一份简单程序只增加一个清晰模块和少量参数；先写输入输出/单位/停止条件，再代码/单元测试。
4. 检查通过后再commit，记录commit或源文件hash到实验目录。失败实验也保留，不改旧日志伪装成功。
5. 等有多个算法模块时，再规划感知/规划/控制/任务包、统一接口/仲裁和回归入口，不现在大改已有两包。

建议后续简易分支：主干保留可复现状态，某实验一个短功能分支，小commit说明“问题、变化、如何验证”。main已在本次GitHub上传整理时建立；功能分支和团队协作约定仍为未来建议。

## 测试分层与命令状态

[S/H 未在本次执行]：

```bash
colcon test --packages-select remote5g_sim remote5g_competition --event-handlers console_direct+
colcon test-result --verbose
```

只在专用空闲实例跑已有仿真回归：`scripts/test_xtnetrc.sh`或 `ros2 run remote5g_competition run_matrix.py ...`。测试脚本有端口检查、超时和拥有进程组清理，仍应先理解它们会如何驱动车辆/操作夹具。不要把所有测试当只读命令。

初始调查没有重新编译；当前频率修订已经colcon build两包，并执行现有两项单元/几何测试及独立运行复测。尚未证明干净新机器可重复构建，也未重跑完整比赛矩阵。

## 最小实验记录模板

```text
实验ID / 日期 / 问题与假设
源码commit或文件hash：
world / vehicle配置 / 相机尺寸 / seed：
ROS_DOMAIN_ID / Gazebo端口 / use_sim_time：
唯一控制源 / 命令 / 初始条件：
预期与通过标准：
数据路径（短bag/日志/图）：
实测指标、失败现象与限制：
结论、知识点、下一步：
```

流程：需求→概念学习→小方案→用户独立实现→仿真测试→数据记录→分析→Git提交→文档归档→下一阶段。

## 本次文档隔离与文件安全

- 初始调查只创建交接资料；本次授权变更清单见snapshot/frequency_update_20261015.json，涉及6个源码/配置/说明文件。几何、原world、执行器限制和依赖环境未改。
- 初始调查保存986个文件hash；当前仅允许频率修订清单中的6项变化，其他初始文件继续核验。交接源副本被COLCON_IGNORE隔离，不参与工作区包发现。
- 只导出允许列表环境变量/源码配置；没读取密码、Token、SSH私钥、浏览器资料或用户完整env。实车PWM说明仅留用途/路径，不导出无关硬件值。
- 轻量交接包排除构建缓存、大DAE/纹理、旧分发zip及bag数据库；资源仅用路径/引用/hash描述，原始小bag留本地。
- `snapshot/collect_*`、`probe_runtime.py`等是本次审计工具，不是新增比赛算法；不会被ROS包构建或常驻运行。运行检查会建自己的仿真并发小指令，未经理解不要在别人在线实例上重复运行。

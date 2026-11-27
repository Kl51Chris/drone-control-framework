# Research handover · MPC vs DeePC under GNSS degradation

交接快照为 2026-11-27，时区 America/Chicago。项目根目录是 /home/chris/drone-project。Phase 5.3.1 已完成。**Phase 5.4 尚未启动，需要用户批准。** 本页是接续工作的上下文入口。依次核对本页的暂定配置、[参数登记表](../research_evidence/parameter_registry.csv)和[分组 CV 报告](../research_evidence/phase531/report.md)。

## 1. Research objective

研究问题是：在 CrazySim 中，从相同的观测与物理控制接口出发，MPC 与 DeePC 在预先定义的 GNSS 退化下，跟踪、控制代价、实时可行性和恢复表现有何不同？

采用 **Shared-Dataset Same-Cost Comparison**：

- MPC 辨识和 DeePC Hankel 使用同一个 phase3-v1 开发数据版本及 episode 级划分。M0 是未用共享数据辨识的物理对照。
- 共有的外环频率、动作接口、物理 Q/R/S、评分权重、参考策略、物理约束、HLC 生命周期和配对扰动条件保持一致。控制器内部可在同一有限预算内选择结构或正则化。
- 真值仅用于离线评分。预测回放可以条件化于已记录的未来输入，但在线控制器不能获得未来真值、噪声、实际未来输入或未经许可的参考。
- 已验证范围是 CrazySim 单机、单 UAV、固定初始 yaw 的 near-hover 实验。DeePC 依赖局部行为表示，MPC 依赖局部线性化。现有数据不能证明大尺度或真机转移性能。
- L0–L3 是对主机外环测量的人工扰动档位。现有结果没有验证正式 GNSS 注入、真实 GNSS 误差分布、outage/bias 恢复或原生 IMU/气压计噪声开启时的鲁棒性。
- 研究里程碑按当前年份登记。**Technical Presentation · 2026-12-03**。**Final Report · 2026-12-13**。具体提交时刻未提供。Jeremy 技术会议材料从[汇报图索引](../research_evidence/presentation_index.md)取用。

[研究方法与数据路线图](shared-dataset.md) · [共享噪声协议](phase5-noise-protocol.md)

## 2. Current project status

| Phase | 已完成和关键发现 | 决策、未解决项 | 报告和原始路径 |
|---|---|---|---|
| 1 | 建立九状态 M0/OSQP 与 HLC 起飞、body-rate 跟踪、HLC 降落。验证悬停、X 阶跃、圆轨迹和主动触发回退。 | 复用共享生命周期。该阶段部分评分靠估计态。 | [测试指南](crazysim-hlc-testing.md) · [基线悬停](../log/20261126_161258_fixed-hover_outer-loop-mpc_crazysim-cf2x-t350-calibrated_production-flight) · [人工回退](../log/20261126_161400_fixed-hover_outer-loop-mpc_crazysim-cf2x-t350-calibrated_production-flight) |
| 2 | 独立 MuJoCo 真值、批量试验、正负 X/Y/Z/yaw、preview 对照、N15/25/40。圆轨迹 preview 将滞后从 .437 s 降至 .092 s。N40 nominal 更准但求解开销更高。 | 已知圆轨迹用 preview，突发阶跃无提前预告。N25 为后续共同开发值。保留一次中断的 horizon 记录。 | [报告](mpc-phase2-results.md) · [preview](../log/mpc-phase2-compare) · [nominal](../log/mpc-phase2-nominal) · [horizon 完整重跑](../log/mpc-phase2-horizon-repeat) · [原中断](../log/mpc-phase2-horizon) |
| 3 | 过滤多轴激励形成 phase3-v1。18 次采集有一次超观测年龄门槛被拒。17 个 episode 为 11 train、3 validation、3 sealed test。近悬停训练深度 40/44/55/59 输入 PE 满行秩。历史基准圆 40 步回放的末端估计态误差范数在 window-local 下由 65.14 降至 9.16 mm。 | DeePC 用 window-local，MPC 用 takeover-relative。局部数据适合开发，扩大包络或 GNSS 退化的充分性仍待验证。 | [质量报告](shared-dataset.md) · [采集](../log/phase3-shared-raw) · [数据 manifest](../log/phase3-v1/manifest.json) · [离线坐标／模型对照](../log/phase3-offline-coordinates/report.md) |
| 4 | DeePC 闭环原型、MPC 历史 bridge、共享日志和人工量测噪声。实现 M0、Hybrid、Joint FIR6 名义对照。 | 当前路径主要证据仍是小幅近悬停。名义试验覆盖不足以判断约束活跃时的实时性。 | [原型指南](phase4-testing.md) · [MPC 模型](../log/phase4-mpc-models) · [DeePC 阶跃](../log/phase4-deepc-steps) · [圆](../log/phase4-deepc-circle) · [回退](../log/phase4-deepc-fallback) |
| 4.1 | 用同一状态及历史输入核对实际 MPC 与独立预测器。2,415 个快照最大差 8.33e-17。36 次配对名义飞行显示，辨识降低部分 effort 和物理代价，M0 的位置 RMSE 在该对照中更小。 | 排除所测路径的预测索引不一致。继续监控低频 yaw 模型偏差。 | [诊断](../log/phase41-diagnosis/report.md) · [两轮](../log/phase41-repeat-1) · [第二轮](../log/phase41-repeat-2) |
| 4.2 | 核对命令、固件、估计器、Euler 和真值符号。Joint FIR6 的有效 yaw DC 增益约 -0.209。N40 有更好的部分跟踪结果，完整输出软约束压力仍有超时。 | 有效辨识延迟包含内环、传输及估计影响。保留 signed yaw 监控，N40 仅作历史和敏感性证据。 | [低频和时域诊断](../log/phase42-diagnosis/report.md) · [N25/40 名义原始](../log/phase42-deepc-nominal) |
| 4.3 | 分轴和低频加权 FIR25 改善离线预测，15 次固定 N25 名义对照没有显示长 FIR 的可重复闭环优势。预分解软约束 QP 加快部分 DeePC 求解，双轴压力仍见超时。 | 结束 MPC 辨识与 FIR 阶数搜索。Joint FIR6 主候选、M0 物理对照，共同 N25，继续 phase3-v1。 | [FIR 与求解诊断](../log/phase43-diagnosis/report.md) · [MPC 对照](../log/phase43-models) · [DeePC](../log/phase43-deepc) |
| 5.1 | 完整 primal-dual QP 保留全部约束。800 个合成 active-output 压力样本均在 12 ms 内。比较 T_ini=6/12/18/25。长历史改善离线 yaw/Z，但缩小 Hankel 列数、恶化条件数。一次 +Y/T18 收到重复时间戳而回退，隔离复跑通过。 | 保留 T6/T12 开发候选。T18/25 为历史诊断。持续活跃约束的完整闭环仍需独立证据。 | [报告](../log/phase51/report.md) · [原矩阵](../log/phase51/nominal) · [复跑](../log/phase51/y-retry) · [压力](../log/phase51/stress.json) |
| 5.2 | 既有 host 噪声定义为 L0–L3。36 次预登记配对全部执行，L2 完整评分 22/24。T12 负 yaw 与 T6 圆轨迹各一次超 12 ms 后回退。 | 配对共享噪声向量已核对。L0/L1 开发、L2 独立压力、L3 协议档位。人工量测噪声与正式 GNSS 退化分开。 | [报告](../log/phase52/report.md) · [原始矩阵](../log/phase52) · [两次失败](../log/phase52/c/010) · [圆轨迹失败](../log/phase52/c/022) |
| 5.3 | 40 次有限 noise-aware 调参均完成正式窗口，16,040 个更新无 QP 超时。降低初始化权重到 .25，筛选 L1 固定代价约下降 78%。2S 降低平均物理代价但四控制器 XY 均变差。 | 暂定 T6 主参考、T12 对照，共用 ρ=.001、β=.25，保留原始 Q/R/S。输出软约束本批未活跃。DeePC 在部分 L1 场景仍有明显较高的 effort/slew 代价。 | [计划和结果](../log/phase53/report.md) · [40 次原始](../log/phase53/runs) · [全量参数表](../research_evidence/regularization_trials.csv) |
| 5.3.1 | 11 个 train episode 五折完整留出。25 格、L0、两组 L1；仅三组进入 L2。全时域 XYZ RMSE 两个 L1 seed 为当前 5.085/5.213、A 1.479/1.518、B 1.616/1.656 mm。A/B 两 seed 各赢五折。 | 当前闭环参考保留。A、B 是待批准的闭环确认候选，ρ 达到网格上限，停止扩网。原 development validation 与 test 都未读取。条件预测优势尚未在闭环确认。 | [CV 报告](../research_evidence/phase531/report.md) · [原始指标](../log/phase531/results) · [预登记 folds](../research_evidence/phase531/folds.json) |

## 3. Current parameter configuration

**Provisional** 是开发配置。**Frozen** 尚不存在。程序默认值与显式研究配置分开记录，执行前需核对 resolved metadata，避免误用默认的 M0 或 β=1。

| 参数 | 当前开发配置及单位 |
|---|---|
| MPC | 主候选 Joint FIR6，30 维实现态（含命令历史和偏置）。M0 是九状态物理对照，也是 CLI 默认。M0 的 hover thrust 为 0.47681863，垂向增益 14.495 m/s² 每单位归一化推力。两者使用 takeover-relative 位置与固定初始 yaw。 |
| DeePC | phase3-v1、16 个保留 latent 方向、固定初始 yaw、每预测窗的局部位置以当前估计位置重锚。缩放用训练通道标准差和原下限。 |
| N 与 T_ini | 共同 N=25（0.50 s）。DeePC T6 主参考、T12 对照。T18/25 仅历史。N40 优化已关闭。 |
| 输出顺序 | x, y, z, vx, vy, vz, roll, pitch, yaw。位置 m，速度 m/s，角度 rad。 |
| 输入顺序 | roll rate, pitch rate, yaw rate, collective thrust 相对悬停的偏差。前三项为 rad/s。 |
| Q | diag(120, 120, 200, 12, 12, 24, 6, 6, 10)，对每个预测步的物理跟踪误差。 |
| R / S | R = diag(2, 2, 1, 12)，S = diag(2, 2, 1, 12)，分别对物理输入和其增量。共同评价以此原始权重对真值及实际发送命令积分，跟踪、effort、increment 分项，除活动评分时间。控制器特有正则项不纳入共同代价。 |
| 正则化定义 | g penalty = λ_g‖g‖²/2，实际 λ_g=(ρ σ_max(H_u_scaled))²。初始化 penalty = β‖e_ini‖²/2，e_ini 为保留的归一化历史输出残差。物理输出 slack 是单独约束变量和惩罚。 |
| ρ / β 参考值 | T6/T12 均用 ρ=.001、β=.25。全 11 个训练 episode 的生产缩放下，λ_g(T6)=.0133075422，λ_g(T12)=.0156936665。代码默认 β=1。 |
| CV 提议仅用于确认 | A: T6、ρ=.01、β=.0625。B: T6、ρ=.01、β=.125。CV 各折实际 λ_g 为 .95133–1.13851。若用同一完整 T6 训练集重建，按现有 σ_max 推得约 1.33075422，运行前应从 resolved 配置/构造结果核对。该数不是已验证的闭环代价或最终参数。 |
| 参考策略 | 已知圆轨迹用 preview。突发阶跃直到实际发生才提供。原 4 cm 半径、12 s 圆是典型调参轨迹，未来独立场景另登记。 |
| 频率与时序 | 主机 50 Hz，标称周期 20 ms。求解接纳阈值 12 ms，与 runner deadline 分别统计。MPC 用 OSQP，DeePC 有单轴筛选和完整 primal-dual 回路。 |
| QP 硬输入限制 | 角速度三个通道各 ±.20 rad/s，绝对 collective thrust [.38,.56]。 |
| QP 软预测状态盒 | 对上述状态顺序的对称上界 [.40,.40,.45,.40,.40,.60,.10,.10,.30]。采用原有物理坐标映射和 slack，不能当作 runner 的硬安全范围。 |
| CrazySim 独立硬安全 | 水平位移 .50 m，相对地面高度 [-.05,.60] m，水平速度 .50 m/s、垂直速度绝对值 .75 m/s、roll/pitch 绝对值 .25 rad。观测年龄不超过 .15 s，工作循环 dt 上限 .10 s。默认最长飞行固定悬停 9.5 s、圆轨迹 30 s，解锁位置跳变 .10 m。降落检查高度 .06 m、垂向速度 .08 m/s。以原配置和逐次 metadata 为准。 |

测量噪声从 body-rate 首次尝试更新开始，包含 bridge。同一组按场景 seed 和更新索引配对。扰动作用于主机 controller observation，HLC、安全监测、机载估计器、独立 MuJoCo 真值走原路径。默认各通道每 tick 独立零均值高斯。噪声为每轴标准差：

| 等级 | 位置 mm | 速度 mm/s | 姿态 mrad | body rate mrad/s | 用途 |
|---|---:|---:|---:|---:|---|
| L0 | 0 | 0 | 0 | 0 | nominal |
| L1 | 1 | 5 | 1 | 10 | 主要开发噪声 |
| L2 | 2 | 10 | 2 | 20 | 独立压力 |
| L3 | 3 | 15 | 3 | 30 | 协议档位，未作为正式闭环证据 |

HLC 按原流程起飞和落地。接管条件包括 HLC 起飞轨迹结束后至少 .4 s、垂向目标误差不超过 .05 m、速度范数不超过 .10 m/s、roll/pitch 不超过 .10 rad。M0 从悬停 seed 初始化输入增量。Joint FIR6 由 M0 先填充 5 个实际发送的历史命令。DeePC 用 MPC bridge 收集 T_ini 个已发送命令和 T_ini+1 个连续观测，保留真实量化推力输入，随后切换。重复、倒序或间隔超过 .04 s 的观测可触发 DeePC 历史失败。求解超时、证书/状态失败或其它 MPCSolveError 在原恢复包络内走 HLC 降落。恢复包络外（速度范数超过 .10 m/s 或 roll/pitch 绝对值超过 .10 rad）触发硬安全路径。预定的 bridge 和正式评分分别记录。

[机器可读暂定配置](../research_evidence/final_configuration.yaml) · [T6 显式配置](../log/phase53/configs/t6-init-quarter-s1.json) · [T12 显式配置](../log/phase53/configs/t12-init-quarter-s1.json) · [rho 推导](../log/phase53/report.md) · [控制器定义](../velocity_control_framework/controllers/outer_loop_deepc.py) · [生命周期与安全](../experiments/run_flight.py)

## 4. Research evidence and retention

从 Phase 5.4 起按[研究报告协议](research-reporting-protocol.md)提交可视化优先的实验报告。

- [参数登记和限制](../research_evidence/parameter_registry.csv)、[决策链 D01–D13](../research_evidence/decision_log.md)、[实验及复现索引](../research_evidence/experiment_index.csv)。失败、未完成、故意回退和真实回退有不同状态。以研究评分窗口及可行性字段为准，流程退出成功不能替代跟踪成功。
- [Phase 5.3 全量逐次指标](../research_evidence/regularization_trials.csv)、[数学与选择依据](../research_evidence/regularization_comparison.md)、[Phase 5.3.1 逐折指标](../research_evidence/phase531/fold_metrics.csv) 和[全部候选](../research_evidence/phase531/candidate_metrics.csv)。
- [主图、附录与证据限制](../research_evidence/presentation_index.md)。部分历史比较使用不同协议，展示时保持 cohort、统计窗口及 metric definition。
- [档案说明与维护清单](../research_evidence/README.md)。原始路径主要在 git-ignored 的 log/。核对本机数据或恢复已校验的 [Phase 5.3](../research_evidence/archive/phase53-complete.tar.gz) 和 [Phase 5.3.1](../research_evidence/archive/phase531-offline.tar.gz) 归档。Phase 5.3 大备份由 Git 忽略。每阶段新增产物后维护登记表、决策和两份索引，保留历史版本。
- 关键失败：[Phase 2 horizon 中断](../log/mpc-phase2-horizon/002)、[Phase 3 拒收](../log/phase3-shared-raw/009)、[Phase 5.1 初始重复观测](../log/phase51/nominal/014)、[Phase 5.2 负 yaw](../log/phase52/c/010)、[Phase 5.2 圆](../log/phase52/c/022)。复跑不能替换这些证据。

## 5. Closed decisions

以下工作已结项。没有新的明确用户批准，后续 agent 不得重新打开：

- MPC system identification 和 FIR 阶数／模型结构搜索。
- DeePC 坐标表示选择、替代 normalization、phase3-v1 数据扩采或重新划分。
- N40 优化。现有 N40 证据留作历史与可能另行批准的敏感性测试。
- 常规 solver optimization。若再次出现可复现、可归因且阻碍后续实验的失败，先保留日志和提交证据给用户。
- 大规模超参数搜索。Phase 5.3.1 已完成 25 格补充验证，ρ 达搜索边界不触发额外搜索。

## 6. Remaining work and approval gates

| 阶段 | 批准后工作及产物 |
|---|---|
| **5.4 Independent confirmation and parameter freeze** | 先提交独立闭环预登记。固定原共同 Q/R/S、N25、phase3-v1、物理及时序标准。保留 Joint FIR6、M0、T6 原闭环参考及用户批准的 A/B，T12 作开发对照按预算决定是否纳入。选未参与调参的新 seed 和独立参考条件，优先悬停、正负 yaw/Z、圆轨迹，关键工况建议至少三次独立配对，L2 压力单列。记录初始/bridge 与正式窗、跟踪、实际物理代价、时序、回退、硬约束和预测输出激活。检查 Joint FIR6 yaw 偏差。CV 不能取代闭环或持续活跃约束验证。独立证据和用户确认都完成后才冻结，写明版本及哈希。 |
| **6.1 Inspection environment and trajectory** | 检查既有 CrazySim/MuJoCo 场景和实际参考接口，定义与研究问题相关的 inspection 路径、运动包络和参考可用性。在两种控制器相同 nominal 条件下确认场景、真值和安全路径。具体场景、尺度和轨迹由后续阶段确认。 |
| **6.2 GNSS degradation implementation** | 在已确认的模拟器/估计器边界设计 outage/bias 注入，登记注入层、轴向、幅值、起止、时长、恢复判据、seed 复现和独立真值。验证未注入与受扰功能及安全守卫。现有外环 iid 噪声不替代该实现。 |
| **6.3 Formal robustness experiments** | 在冻结参数与预登记 GNSS 场景下执行配对实验，保留全部失败、超时和回退。分开报告 tracking、恢复、实际物理代价、可行性和不确定性。不得据此倒推调整控制器。 |
| **Final analysis and reporting** | 按独立 episode 计分，保留配对、失败分母及适用边界。更新研究证据与图表，服务 Jeremy 技术讨论、12 月 3 日 Technical Presentation 和 12 月 13 日 Final Report。 |

**新会话第一步：** 在用户批准 Phase 5.4 前，只核对交接与现有证据。批准后先报告 Git 状态、预登记矩阵与新 seed。不要把现有名义和 CV 数据当作独立闭环确认集。

## 7. Repository, environment and reproducibility

快照开始时 branch 为 feat/crazysim-integration，HEAD 为 e85310aa739225f98a1bed9a996f71cf41f770ba，领先已记录 remote-tracking branch 3 个提交。以下状态列的是交接文档写入前：

| 状态 | 路径 |
|---|---|
| 已跟踪的改动 | doc/phase4-testing.md、velocity_control_framework/controllers/outer_loop_deepc.py、velocity_control_framework/tests/test_phase4_controllers.py |
| 未跟踪 | CLAUDE.md、doc/phase5-noise-protocol.md、research_evidence/、velocity_control_framework/controllers/deepc_soft_qp.py、velocity_control_framework/tests/test_deepc_initialization_weight.py、velocity_control_framework/tests/test_deepc_soft_qp.py |
| 本次新增 | doc/research-handover.md |

Phase 5.1–5.3 的实现与档案目前存在于工作区，不能假定 HEAD 包含全部实验代码。本轮未提交。开始新阶段时重新记录状态，保护已存在的改动和 ignored logs。不要用 reset、clean 或覆盖输出目录来准备试验。

本地 Linux，项目与 CrazySim Python 均为 3.10.12。项目环境 .venv，CrazySim 环境位于 /home/chris/CrazySim/.venv。观测版本为 NumPy 2.2.6、SciPy 1.15.3、OSQP 1.0.4、cflib 0.1.32、matplotlib 3.10.8、pytest 7.4.4，CrazySim 的 MuJoCo 3.17.0。CrazySim/firmware 当前 HEAD 为 3ec8b55/aa6571dc。已有 SITL 路径为 /home/chris/CrazySim/crazyflie-firmware/sitl_make/build/cf2。正式试验沿用 CrazySim CF2X T350、1 ms physics dt、独立进程与 UDP 端口、主机 50 Hz。锁定依赖见 [Ubuntu](../requirements-ubuntu-lock.txt)、[WSL](../requirements-wsl-lock.txt)及[环境指南](getting-started.md)。

入口和命令。从项目根目录运行，所有复跑用新输出目录及空闲端口。本轮没有执行下列实验命令。

    cd /home/chris/drone-project
    export OPENBLAS_NUM_THREADS=1 PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1
    git status --short --branch
    .venv/bin/python research_evidence/verify_archive.py

    # 既有测试命令。最近记录为 Phase 5.3 的 429 项通过
    .venv/bin/python -m pytest velocity_control_framework/tests calibration/tests -q -p no:cacheprovider

    # 已存在的离线 CV 数据配方。仅打开训练集，输出到新目录
    .venv/bin/python log/phase531/cross_validate.py log/phase531/folds.json log/phase531/replay-results
    .venv/bin/python log/phase531/figures.py log/phase531/replay-results log/phase531/folds.json log/phase531/replay-figures

    # 历史 Phase 5.3 工作流，仅供授权后的历史复跑
    .venv/bin/python log/phase53/run.py log/phase53 log/phase53/handover-replay --port 26360
    .venv/bin/python log/phase53/audit_results.py log/phase53 log/phase53/handover-replay /tmp/phase53-handover-audit.json

生产飞行配置入口见 [experiments/run_flight.py](../experiments/run_flight.py)。已批准的混合控制器实验可参考 [Phase 5.3 顺序批量脚本](../log/phase53/run.py)，通用单控制器矩阵使用 [tools.run_crazysim_batch](../tools/run_crazysim_batch.py)，其子进程记录实际 firmware、simulator、flight 命令。手动三终端与 GUI 说明见 [CrazySim 测试指南](crazysim-hlc-testing.md)。正式飞行仅限已授权的 CrazySim，硬件执行属于独立审批。

## 8. Experimental integrity and anti-tampering

本节继承原研究规则。可信、可复现的结果优先于实验通过率和性能。失败是可接受的研究结果。

1. **测试守恒。** 未经用户明确批准，不放宽阈值或容差，不改 expected values，不删、跳过、禁用、弱化、减少测试或预定重复数，不变更条件绕开已知失败，不改失败标签，不以 mock、stub、hard-coded 输出或场景特判绕过被测功能。可新增测试、诊断或日志。若怀疑原测试有错，保留原测试与失败，提交依据、拟议修改及影响，等待批准。
2. **评价守恒。** 不擅改 12 ms 求解预算、50 Hz、物理/安全约束、阶段约定的共享 Q/R/S、真值对齐与评分方法、tracking error、physical cost、settling time、正式评分时间窗、噪声分布/seed/配对协议、split 或 fallback / deadline 统计定义。研究某项权重时，先登记候选和不变的物理评价权重。禁止缩窗、删不利时间段或重定义成功。
3. **试验登记与失败。** 开始前保存 Git HEAD 和 dirty 状态、测试配置与数量、数据 manifest、候选与参数、seed、预定矩阵、排除和验收规则及上限。每个原定试验都计入状态与安全分母，只有预登记理由才可剔除正式评分，并说明。保存失败、超时、中断、回退、部分日志、实际配置、版本和 replay 命令。允许复跑时保留首次失败、原 seed 和条件，单列复跑原因与结果。不能用成功复跑覆盖首次，更不能换 seed 找有利结果。
4. **数据保护。** phase3-v1 及划分不可覆盖、编辑或重划。完整 episode 是独立单元。锁定的三个 test episode 保持封存，包括其处理文件和对应原始结果。开发/调参/选择禁止读取。既有三个 validation episode 仍是开发验证。5.3.1 使用 11 个 train episode 在内部交叉验证。线上不接入真值、未来观测、未来噪声或真实未来输入，正式 GNSS 结果不用于反向调参。
5. **结项与阻塞。** 逐项说明修改的源代码与测试、diff 摘要、测试数量变化、原计划与执行差、全部失败和复跑、任何评价或配置变化、对应日志与命令。预算内无法达标时，停止优化，保留结果和可能原因，提出有限后续方案并等待用户决定。无证据的项标为 unavailable 或 not yet verified。

[证据完整性与归档核对说明](../research_evidence/README.md#retention-and-integrity) · [已有预登记实例](../research_evidence/phase531/plan.md)

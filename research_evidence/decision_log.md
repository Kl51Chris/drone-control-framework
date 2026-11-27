# 研究技术决策

[参数表](parameter_registry.csv)区分已确认的开发决策、暂定参数和历史候选。正式冻结尚未发生。按[实验 ID](experiment_index.csv)查找原始定位、记录版本及复现命令。各组数字保留原有回放、闭环或压力协议。

## D01 · MPC 物理与辨识模型

- **Problem**　是否能用可解释的物理 MPC 作对照，并检验辨识的收益。
- **Hypothesis**　近悬停九状态模型适合作为基线，有效内环和推力响应可能改进预测及代价。
- **Experiment**　Phase 1 验证 HLC 接管、悬停、X 阶跃、圆轨迹和人工故障。Phase 3–4.1 比较 M0、Hybrid 和 Joint FIR6，匹配未来输入和共同物理代价。
- **Evidence**　Phase 4.1 有 36 次名义对照。N40 X 阶跃中 M0/Hybrid/Joint 的 XYZ 真值 RMSE 为 15.74/16.04/16.08 mm。同组成本／秒为 .05560/.04903/.04918。
- **Decision**　Joint FIR6 为主要辨识候选，M0 为物理对照。MPC 保持 takeover-relative 和固定 yaw。
- **Limitation**　较低物理代价可伴随较高跟踪误差。Joint FIR6 仍需监控低频 yaw 增益偏差。证据限于 CrazySim 近悬停。

[Phase 1 命令及基线](../doc/crazysim-hlc-testing.md) · [Phase 4.1](../log/phase41-diagnosis/report.md) · [代价分解](../log/phase42-diagnosis/mpc-phase41-cost.json)

## D02 · FIR 记忆长度

- **Problem**　FIR6 是否遗漏慢轴响应，延长各轴记忆是否有闭环收益。
- **Hypothesis**　yaw 和 thrust 可能需要不同记忆。较长 FIR 可以改善回放预测。
- **Experiment**　Phase 4.2–4.3 比较 FIR4–25 和分轴布局。共享 train/validation。五种模型在固定 N25、Q/R/S、悬停、X 阶跃和圆轨迹中各运行一次。
- **Evidence**　M0/Joint/[10,10,16,12]/低频加权 FIR25 实现维数为 9/30/54/106，40 步末端位置误差为 9.37/2.44/1.65/1.22 mm（各轴 RMSE 的平方平均根）。同批 X 阶跃成本／秒为 .0559/.0520/.0531/.0536。
- **Decision**　保留 Joint FIR6。分轴和 FIR25 留作历史对照。停止模型结构搜索。
- **Limitation**　每场景一次。增广维数包含历史命令与偏置。validation 被多次用于开发。

[分轴证据和复现](../log/phase43-diagnosis/report.md) · [逐轴响应](../log/phase43-diagnosis/orders.json)

## D03 · 预测时域 N

- **Problem**　更长的预测是否改善跟踪且满足共享实时预算。
- **Hypothesis**　长时域可以减小滞后，但会增加 QP 工作量。
- **Experiment**　Phase 2 固定 M0、preview、权重及约束，比较 N15/25/40，圆轨迹和 X 阶跃各三个重复。Phase 4.2–4.3 对 DeePC 比较 N25/40 的预测、名义运行和活跃输出约束压力。
- **Evidence**　MPC 圆轨迹 X RMSE 6.8/4.9/3.8 mm，run-level QP P99 均值 .69/1.20/2.05 ms。X 阶跃 N25 为 0/3 settled，N40 为 3/3。Phase 4.3 双轴压力 N25/N40 为 2/30、1/30 超时。
- **Decision**　共同开发 N25（0.50 s，50 Hz）。N40 留作敏感性检查。
- **Limitation**　闭环 nominal 与离线活跃约束压力协议不同。MPC 的收益不能单方面决定共享时域。

[Phase 2 证据及复现](../doc/mpc-phase2-results.md) · [Phase 4.2](../log/phase42-diagnosis/report.md) · [Phase 4.3](../log/phase43-diagnosis/report.md)

## D04 · Reference preview

- **Problem**　已知圆轨迹是否应提供未来参考。
- **Hypothesis**　在相同 horizon 下，preview 能减少相位滞后。
- **Experiment**　Phase 2，N25、半径 0.10 m、15 s 一圈，每个策略三次。
- **Evidence**　Constant 与 preview 对应的 X/Y 真值 RMSE 为 16.0/14.2 与 4.6/4.1 mm。滞后为 .437 与 .092 s。QP P99 均值均为 1.03 ms。
- **Decision**　已知圆轨迹用 preview。突发阶跃按发生时刻提供。
- **Limitation**　结果限于匹配圆轨迹及相同可用参考信息。

[Phase 2 结果和命令](../doc/mpc-phase2-results.md) · [逐次结果](../log/mpc-phase2-compare/summary.json)

## D05 · 共享数据和 Hankel PE

- **Problem**　局部数据能否支持 MPC 辨识和 DeePC。
- **Hypothesis**　滤波多轴激励及 episode 级划分可提供可审计的局部 PE。
- **Experiment**　从历史轨迹和单轴 pilot 到滤波激励，按时序、覆盖、饱和、频谱、PE 和独立验证筛查。
- **Evidence**　18 次采集的一次因观测至发送年龄 41.73 ms 超过既有 40 ms 门槛被拒收。17 个 admitted episode 由 11 train、3 validation、3 sealed test 构成。深度 40/44/55/59 的输入数值秩为 160/176/220/236，均满行。
- **Decision**　保留 phase3-v1 和现有划分，作为共同开发数据。
- **Limitation**　水平覆盖约 ±3 mm。数值满秩无法证实真实系统阶数、扩大包络的预测稳定性或正式噪声性能。

[采集与质量报告](../doc/shared-dataset.md) · [汇总](../log/phase3-v1/quality.json) · [划分 manifest](../log/phase3-v1/manifest.json)

## D06 · DeePC 坐标表示

- **Problem**　位置平移是否造成 Hankel 外推。
- **Hypothesis**　统一的一窗一锚可减少外推，并保持绝对参考转换清晰。
- **Experiment**　Phase 3 同历史起点和实际未来输入，比较 takeover、window-local 和 increments，固定 yaw，分析 N25/40。
- **Evidence**　历史基准圆 N40、旧离线 ridge factor 1e-3 的末端估计态 XYZ RMSE 范数为 65.14/9.16/9.39 mm。共同位置尺度的对照也保留重锚收益。
- **Decision**　DeePC 用 window-local 位置。MPC 继续用 takeover-relative，并保留物理约束投影。
- **Limitation**　这是同输入历史回放。在 iid 噪声下，重锚使旧位置项方差乘 2，窗口内相关系数为 0.5。

[坐标证据和复现](../log/phase3-offline-coordinates/report.md) · [噪声诊断](../log/phase53/diagnosis.json)

## D07 · 初始化长度 T_ini

- **Problem**　延长历史是否改善预测、求解和闭环。
- **Hypothesis**　较长历史可描述慢响应，但会减少可用列并恶化条件数。
- **Experiment**　Phase 5.1 比较 6/12/18/25 的匹配预测、PE 和闭环子集。Phase 5.2–5.3 确认 6/12。
- **Evidence**　四窗口 Z 末端误差 .880/.433/.328/.287 mm，yaw 4.295/1.941/.715/.462 mrad。联合有效条件数 1698/2133/3362/5961。Phase 5.3 第二 seed L1 圆轨迹成本／秒为 T6 .02834、T12 .05217。
- **Decision**　暂定 T6 主候选，T12 作对照。T18/25 保留为历史诊断。
- **Limitation**　离线优势与闭环优势未一致。Bridge 影响进入评分窗时的状态。

[Phase 5.1 与复现](../log/phase51/report.md) · [Phase 5.3](../log/phase53/report.md)

## D08 · DeePC ridge 和初始化惩罚

- **Problem**　如何分开控制 g 幅度与受扰历史的拟合。
- **Hypothesis**　降低初始化残差权重可以降低噪声引起的实际控制代价。
- **Experiment**　固定 N25 和共同 Q/R/S，每个 T 比较 (rho,beta)=(.001,1)、(.003,1)、(.001,.25)，先 L0/L1、再第二 seed、另做 L2 压力。
- **Evidence**　T6 三组 L1 平均固定代价／秒为 .12912/.10475/.02799，T12 为 .20566/.16889/.04494。第二 seed 六次、L2 四次均完成原评分窗口。
- **Decision**　暂定 rho=.001、beta=.25。实际 lambda_g=(rho*sigma_max(H_u_scaled))²。
- **Limitation**　Phase 3 历史离线 rho 使用联合输入/历史输出 regressor。当前与旧值不可直接比较。归一化初始化残差单独记录。

[40 次全量表](regularization_trials.csv) · [Phase 5.3 复现](../log/phase53/report.md)

## D09 · 共同 Q/R/S

- **Problem**　加强输入 slew 惩罚是否兼顾物理代价和跟踪。
- **Hypothesis**　2S 会减少输入变化，可能牺牲 XY。
- **Experiment**　四控制器配对 L0/L1 圆轨迹，控制器侧为 S 和 2S，Q/R 固定。全部用原始共同 Q/R/S 计算实际评价代价。
- **Evidence**　固定平均代价从 .012945 降至 .009927／秒。四种控制器平均 XY RMSE 都增加。
- **Decision**　按预登记条件保留原始 S，全部物理权重暂定。
- **Limitation**　只测试了一个 S 方向。Q/R 未做独立 sweep。

[配置与选择](../log/phase53/runs/decisions.json) · [配对矩阵](../log/phase53/runs/shared/matrix.json)

## D10 · 共享测量噪声

- **Problem**　如何保持两类控制器接受可复现的共同量测扰动。
- **Hypothesis**　以场景 seed 和控制步索引 host 噪声，可直接验证配对。
- **Experiment**　Phase 5.2 先完成 T12 nominal，然后用四种控制器跑 L2 pilot。L0–L3 使用现有 iid Gaussian 发生器。
- **Evidence**　36 次原始实验，22/24 L2 完整评分。共同索引的扰动逐项相同。L2 水平位置幅度约 2.7–3.1 个 DeePC 训练尺度。
- **Decision**　L0/L1 开发、L2 独立压力、L3 协议档位。
- **Limitation**　原生 CrazySim IMU/气压计噪声关闭。人工外环测量噪声不标识真实 GNSS 误差。

[协议](../doc/phase5-noise-protocol.md) · [审计和复现](../log/phase52/report.md)

## D11 · 求解策略与预算

- **Problem**　约束活跃或噪声下能否在 12 ms 内求解。
- **Hypothesis**　紧凑 Newton 系统可以减少工作量，同时保留完整约束。
- **Experiment**　Phase 4.3 筛选式 QP。Phase 5.1 完整 primal-dual 路径和完整 OSQP 对照。Phase 5.2/5.3 登记各自闭环统计。
- **Evidence**　Phase 5.1 离线活跃输出 800 次均在 12 ms 内，最大 9.21 ms。Phase 5.2 L2 两次 12.020/12.655 ms 超时并回退。Phase 5.3 40 次、16,040 次更新无超时，最大 5.82 ms，预测输出软约束激活为零。
- **Decision**　保留约束、12 ms 接纳预算及 HLC 回退。无额外求解器搜索。
- **Limitation**　离线压力和两个闭环阶段协议不同。Phase 5.3 不覆盖持续输出约束活跃。

[Phase 5.1](../log/phase51/report.md) · [首次失败](../log/phase52/c/010) · [第二次失败](../log/phase52/c/022) · [Phase 5.3](../log/phase53/report.md)

## D12 · GNSS 退化参数

- **Problem**　如何公平评估 outage、bias、跟踪和恢复。
- **Hypothesis**　锁定共同配置后，预登记的配对故障脚本可比较各控制器响应。
- **Experiment**　Not yet verified。正式 GNSS 退化未运行。
- **Evidence**　Unavailable。现有 pilot 注入普通外环测量噪声。
- **Decision**　待登记注入层、轴向、幅值、起止时刻、时长、恢复规则、配对种子及判据。
- **Limitation**　尚无正式 GNSS 故障性能或恢复时间数据。

[路线图](../doc/shared-dataset.md) · [暂定配置](final_configuration.yaml)

## D13 · DeePC 正则化的分组交叉验证

- **Problem**　Phase 5.3 当前设置是否处于离线预测较稳定的参数区域。
- **Hypothesis**　预测误差对受扰历史拟合的依赖可通过增加 λ_g/β 降低，过度正则可能牺牲名义精度。
- **Experiment**　11 个 train episode 五折各留出一次。固定 T6/N25、原归一化与物理配置，25 格、L0 及两组 L1 seed。每折重建模型；L2 只评当前和两项入围。用已记录实际输入条件化，用独立 MuJoCo 真值评分。
- **Evidence**　五折深度 40 均为 160/160。当前设置同两组 L1 的等 episode XYZ 误差为 5.085/5.213 mm。A（.01/.0625）为 1.479/1.518 mm，B（.01/.125）为 1.616/1.656 mm。两者在两组 L1 各赢 5/5 folds。
- **Decision**　保留当前闭环参考，A/B 等待授权进入 Phase 5.4 确认。ρ 达网格上界，停止扩大网格。正式参数维持暂定。
- **Limitation**　条件预测精度不能代表闭环性能。L0 对最强相对正则出现代价，两个 seed 共享同一组 episode 与折；不能作为独立飞行重复。

[Phase 5.3.1 报告](phase531/report.md) · [五折与种子](phase531/folds.json) · [逐折指标](phase531/fold_metrics.csv) · [原始输出](../log/phase531/results)

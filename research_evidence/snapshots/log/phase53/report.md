# Phase 5.3 有限预算的共同降噪调参

## 固定配置与稽核

起始 Git 为 `e85310aa739225f98a1bed9a996f71cf41f770ba`。已经存在的
Phase 5.1/5.2 工作区状态见 `audit/start-status` 与 `audit/start.diff`。
本轮保留 Joint FIR6 和 M0、phase3-v1、N=25、50 Hz、12 ms 及既有
测量噪声协议、物理约束与评分窗口。未读取锁定测试集或重开 MPC
辨识。没有扩大数据集、变更坐标或增加滤波器。

相对于起始快照，生产代码仅修改
`velocity_control_framework/controllers/outer_loop_deepc.py`，增加正的
`initialization_weight`，默认 1.0，以及对应的诊断代价。
`velocity_control_framework/tests/test_deepc_initialization_weight.py`
新增五项检查，原测试文件未修改。协议文档增加参数定义。
现有评分代码、noise injector、MPC、专用 QP 求解器和数据集清单
与起始 SHA256 一致。借用 Phase 5.1 评分入口的本地副本仅加固定
评价权重入口，五条历史用例的旧权重评分逐项完全相同，包含回退日志。

## 目标函数和噪声分析

记预测输出 `y`、物理参考 `r`、输入 `u`、输入差分 `du`，
归一化初始残差 `e_ini = Yp_scaled g - yp_scaled`，
以及预测输出软约束 slack `s_y`。求解器目标是

```text
J = 0.5 [ ||y-r||_Q² + ||u||_R² + ||du||_S²
          + beta ||e_ini||² + lambda_g ||g||² ]
    + 100 sum(s_y) + 0.05 ||s_y||²
lambda_g = (rho * sigma_max(H_u_scaled))²
```

各物理权重在 horizon 内重复，推力在原悬停工作点居中。
训练输入和输出按各通道训练标准差缩放，标准差下限分别为
1e-5 与 1e-4。约束和 Q/R/S 仍作用于恢复物理单位后的输出与输入。
`rho` 为配置里的 `regularization`，`beta` 为新增的初始化权重。
归一化输入 Hankel 限制过去/未来输入必须匹配；g 在保留的缩减
子空间中运算。初始化残差的 51（T6）或 105（T12）个有效量采用
平方和。预测状态软约束的 slack 是独立项。

| T_ini | sigma_max | 基线 lambda_g | rho=0.003 时 lambda_g |
| --- | ---: | ---: | ---: |
| 6 | 115.358 | .013308 | .119768 |
| 12 | 125.274 | .015694 | .141243 |

对每个固定 yaw 局部位置轴，时刻 k 的旧历史项的噪声为
epsilon(k-j)-epsilon(k)。iid 噪声下方差为 2 sigma²，不同旧项间
协方差为 sigma²、相关系数为 .5。当前项恒为零，并已从 Hankel
拟合行剔除。若沿控制时钟观察同一固定滞后 j 的差分，滞后 j 的
相关系数为 -.5，其余非零滞后为 0。它来自窗口差分，原始注入仍是 iid。

使用预定种子在三个 validation episode 的相同起点逐类注入 L1，
固定未来输入和参考，计算缩减预测与无约束首步解的局部变化。
不作为名义闭环测量，也不反向选择参数。

| L1 单通道族 | T6 / T12 归一化初始残差 RMS | T6 / T12 首步 yaw 指令变化 RMS，rad/s | T6 / T12 首步归一化推力变化 RMS |
| --- | ---: | ---: | ---: |
| 仅位置 | .814 / .876 | .126 / .190 | .0178 / .0339 |
| 仅速度 | .561 / .606 | .0667 / .0797 | .0329 / .0402 |
| 仅姿态 | .137 / .164 | .0209 / .0542 | .0080 / .0102 |

位置噪声对这组归一化残差和 yaw 指令的响应最大，速度噪声对推力
指令变化最大。上述无约束动作响应是局部灵敏度，实际发送的命令
仍受硬约束和闭环反馈限制。实验选择规则在 `plan.md`。

## DeePC 有限搜索

初筛 18 次（6 参数配置 × 3 场景）全部有效。每个参数组的下表
代价为 L1 悬停与圆轨迹的等权平均，每秒归一化，以原始共同物理
Q/R/S 计算。g 与初始化罚项单独记录。

| T_ini | 默认 rho=.001, beta=1 | g-only rho=.003, beta=1 | init-quarter rho=.001, beta=.25 |
| --- | ---: | ---: | ---: |
| 6 | .12912 | .10475 | **.02799** |
| 12 | .20566 | .16889 | **.04494** |

降低 beta 使未加权的初始化残差略增。两个窗口的实际控制 effort
和输入差分代价下降。仅加强 g 惩罚的初筛收益较小。

两种窗口各保留 init-quarter 一组配置。第二 seed 六次确认全通过。

| 确认场景 | T6 代价／s | T12 代价／s | T6/T12 XY RMSE, mm |
| --- | ---: | ---: | ---: |
| L1 悬停 | .02962 | .04776 | .85 / .57 |
| L0 圆轨迹 | .00323 | .00501 | 2.52 / 3.96 |
| L1 圆轨迹 | .02834 | .05217 | 2.33 / 4.19 |

预留 L2 只用于压力评价。两种窗口的负 yaw 和圆轨迹四次都完成
正式窗口，未观察到回退或 QP/runner 超时。负 yaw 目标为 -5°，
T6/T12 yaw RMSE 为 4.19°/3.90°。圆轨迹 XY RMSE 为 2.94/4.23 mm，
物理代价为 .10383/.15212 每秒。四次 QP 最大耗时为 5.35 ms。
双窗口仍有不同跟踪取舍。这四个结果不用于替换或重排初筛候选。
本轮样本数量不支持对压力失败概率作强结论。

## 共同物理权重对照

原始 S 与 2S 的优化器使用相同 Q、R、物理范围和参考。
所有实际代价均使用固定的原始评价 Q/R/S。L0/L1 圆轨迹均同
seed 53021。默认 S 的四组 DeePC 确认结果按预登记复用，另新增
12 次飞行，形成每组八个对照分数。两组均通过安全与时序筛选。

| 控制器 | 固定评价物理代价／秒，L1 原始 S → 2S | L0/L1 平均 XY RMSE，原始 S → 2S（mm） |
| --- | ---: | ---: |
| M0 | .00495 → .00419 | 3.38 → 3.50 |
| Joint FIR6 | .00508 → .00430 | 3.67 → 3.77 |
| DeePC T6 | .02834 → .01878 | 2.42 → 2.56 |
| DeePC T12 | .05217 → .03793 | 4.08 → 4.39 |

八个分数的固定平均代价从 .012945 降到 .009927（约 23%）。
四种控制器的平均 XY RMSE 均上升。依据预登记的“不恶化 XY”
条件，推荐原始 S。较强的 slew 惩罚是本轮的平滑性取舍记录。
原始 S 下，L1 圆轨迹 T6 的加权 effort/输入差分代价为 .01131/.01413，
Joint FIR6 为 .00099/.00152。T6 仍有明显较高的输入侧代价。

## 实时性、安全与后续独立验证

40 次原始飞行全达到其正式窗口。实际顺序为初筛 18、第二 seed
确认 6、预留 L2 4、共同权重新增 12，没有中断、重跑或预定候选
缺漏。16,040 个控制更新均未记录 QP 超时；runner deadline、HLC
回退、硬输入和真值包络违规及预测日志丢包均为零。全阶段 QP
最大 5.82 ms，runner 工作最大 8.10 ms。记录到 4,311 次预测输入
接近边界，预测输出软边界激活为零。本轮结果验证了这些受扰
闭环场景，不覆盖持续输出约束激活。没有触发重新优化求解器的条件。
所有 8 个配对组日志与固定 seed 的重放一致。

推荐开发配置为 Joint FIR6 主 MPC、M0 对照、DeePC 主候选 T6，
备用 T12。两种 DeePC 均用 rho=.001、beta=.25，保留 N=25 和
原始共同 Q/R/S。T6 在两组 L1 圆轨迹试验的实际物理代价均低于
T12，同时保留 T12 在部分姿态跟踪上的证据。

独立闭环确认前应锁定候选、物理评价权重和可行性判据，登记新
seed、正负 yaw/Z、悬停以及新的圆轨迹条件。建议每个关键工况
至少三个独立配对，依次比较 L0/L1 并单列 L2 压力结果。
统计以 episode 为单位，保留所有超时与退出并单独报告约束
激活；继续监测 Joint FIR6 的低频 yaw 偏差。此后才决定冻结共同
配置。最终测试数据保持封存，GNSS 退化留给冻结后的评价。

## 复现

从项目根目录开始。重放使用新目录与空闲端口。

```bash
export OPENBLAS_NUM_THREADS=1 PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1
.venv/bin/python log/phase53/diagnose.py /tmp/phase53-diagnosis.json
.venv/bin/python log/phase53/run.py log/phase53 log/phase53/runs-replay --port 26360
.venv/bin/python log/phase53/audit_results.py log/phase53 log/phase53/runs-replay /tmp/phase53-replay-audit.json
.venv/bin/python log/phase53/plot.py log/phase53/runs-replay /tmp/phase53-replay.png
.venv/bin/python -m pytest velocity_control_framework/tests calibration/tests -q -p no:cacheprovider
```

测试从 424 增至 429，全部通过。实际矩阵、模型参数、日志和
逐场景结果在各 `runs/` 阶段目录。审计和图表输出为
`results-audit.json` 与 `tuning-comparison.png`。

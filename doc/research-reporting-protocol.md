# Research Reporting Protocol

从 **Phase 5.4** 起，项目所有实验报告采用可视化优先的格式。同一协议适用于阶段总结、Technical Presentation 和 Final Report 中引用的研究图表。[Experimental Integrity and Anti-Tampering Rules](research-handover.md#8-experimental-integrity-and-anti-tampering) 继续生效。

## 1. Executive summary

开头用简短文字回答五个问题：

1. 本轮做了什么？
2. 主要结果是什么？
3. 哪些假设得到支持？
4. 哪些问题仍未解决？
5. 哪些决定需要用户批准？如果没有，明确说明。

将实现细节、完整参数和例外记录放到相应章节。

## 2. Required visualizations

按实际实验内容选图。一张合并图可同时满足多项要求，优先解释研究问题。

| Experiment type | Required visualization |
| --- | --- |
| Parameter sensitivity | Validation curves 或 heatmaps，标记当前配置和入围候选 |
| Controller comparison | Grouped bars 或 paired scatter，保留配对关系 |
| Repeated experiments | 逐次观测与汇总统计，标明独立重复数 |
| Tracking performance | Reference 与 ground-truth trajectory 对照 |
| Time-domain response | State 与 control input 时间序列 |
| Noise robustness | Performance versus noise level，区分不同噪声协议 |
| Physical cost | Tracking、effort、slew 分解，跨控制器沿用共同评价权重 |
| Solver performance | ECDF、boxplot 或 timing distribution，标记预算线与超时 |
| GNSS degradation | Fault interval、tracking error 和 recovery curves |
| Inspection mission | Top-view trajectory 与 degradation region |

## 3. Figure requirements

- 使用英文标题、坐标轴和图例，标注物理单位、实验条件、独立样本数及参数配置。用图注补充条件和样本分母。
- MPC 与 DeePC 在图表间保持一致的颜色和线型。默认 MPC 为蓝色实线（`#2864A0`），DeePC 为绿色虚线（`#00866A`）。同类模型、配置和个别 run 用标记或明暗区分，并保持控制器家族的编码一致。
- 可直接比较的子图使用相同坐标范围、刻度及单位。不同评价协议、窗口或指标分开标注。
- 保留 individual run observations。汇总统计必须对应明确的独立样本分母。置信区间仅在样本数和统计假设支持时展示，注明方法和统计单位。
- 在对应图中清楚标记失败、fallback、deadline miss 和 constraint violations。没有完整评分窗的 run 单列状态，不填为零或完整成功；原始部分轨迹保留。
- 禁止截断不利数据、改变评分窗口、隐藏异常或用平均值掩盖失败。使用既有物理限制、真值评分和失败统计定义。

每张关键图保留四项可追溯产物：

| Artifact | Requirement |
| --- | --- |
| PNG | 300 DPI，可直接嵌入报告 |
| Vector | PDF 或 SVG，供报告和演示排版 |
| Figure data | 对应的 CSV 或 JSON，保留实验 ID、条件、逐次观测和失败状态 |
| Plotting script | 可从对应数据复现图表，记录运行命令与依赖 |

## 4. Figure interpretation

每张主要图附三条短说明，按以下顺序书写：

- **Observation** — 报告图中的数值、趋势和异常，注明适用分母或对照条件。
- **Interpretation** — 给出可能的数学或控制理论解释，标明解释何处仍为假设。
- **Limitation** — 说明图不能证明的结论、适用范围及仍待验证的问题。

## 5. Report structure and handoff

完整 Markdown 报告按以下顺序组织：

1. **Executive summary**
2. **Experimental configuration**
3. **Key visualizations**
4. **Quantitative results**
5. **Interpretation**
6. **Failures and limitations**
7. **Recommended decision**
8. **Reproducibility and audit appendix**

在 **Key visualizations** 中用 Markdown 图片语法直接嵌入 PNG，紧随 Observation、Interpretation 和 Limitation，并链接矢量图、对应数据和脚本。轮末会话总结优先提供 **3–6 张**最关键的图表路径及各自一句解释。附录记录预登记矩阵、已执行和缺失的试验、原始路径、配置、评分口径、Git/环境和复现命令。

每阶段持续更新 [research_evidence](../research_evidence/README.md) 和 [presentation_index.md](../research_evidence/presentation_index.md)。将关键图分为主汇报图和附录图，保留原始结果、失败、复跑、历史配置及其状态。

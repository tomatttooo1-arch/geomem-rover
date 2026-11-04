
只用宿主 CPU，不需要 ROS、GPU、FPGA 工具或板子。依赖 C++14、CMake、Python 3.8+、numpy、scipy、matplotlib。
在仓库根运行（正式执行前先提交并打实验标签）：

```bash
python3 -m tools.planner_model.run --trace ${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin --out runs/planner-model-20261102
```

输入需保留相邻的 `formal.bin.json`，以区分 rec30、TUM、synthetic。大输入及输出均留 `runs/`，不进入 Git。
`--out` 必须是新目录或空目录，复跑请另指定路径。输出含所有成功/失败查询、路径、控制结果、图来源/哈希、五张最差案例 PNG 与分层汇总。
C++ 单独编译：`cmake -S tools/planner_model -B runs/planner-build -DCMAKE_BUILD_TYPE=Release && cmake --build runs/planner-build -j2`。

位势 float64 与 uint32 都累加 NavFn 已截断的单格代价，不是拿不相同的输入代价验证定点一致。
uint32 最大势能上界 `(W*H-1)*251`，400² 约 4.02e7，不能用 uint16。无穷 sentinel = UINT32_MAX/4。
8 邻同权按任务书文字实现；这是图上的加权波前，不是 NavFn 的连续 Eikonal 二次插值。
正反扫描遍历全部格与全部合法邻居，收敛数包含最终无更新一轮。路径提取和平滑在宿主完成，硬件拍数估计不含它们。

来源与贡献：

| 类别 | 内容 |
|---|---|
| 厂商/开源 | `vendor/navfn.cpp` / `navfn.hpp`：Nav2 Humble BSD；提交与网址见 upstream.txt。ROS 包装参考文件 Apache-2.0，仅作审计。rclcpp shim 只屏蔽日志。 |

NavFn 包装保持真实 Humble Dijkstra / round worldToMap / mapToWorld 无半格偏移 / 反向提取 / 容差方形扫描 / 末端处理。
为满足任务书「致命目标失败」，两方入口额外拒绝目标 cost>=253；原生 NavFn 的容差行为另外检测，不将这个限制冒充原生行为。
安全评分检查完整线段与闭合格方块，含角接触，默认不清起点的原图作为评分依据。外圈与 NavFn 一样关闭。
离障中位数的 NavFn 侧采用所有 NavFn 成功对，波前侧采用其成功路径，不能因波前失败而删去 NavFn 样本。
离障检查完整线段到 cost>=253（含未知/膨胀障碍）格中心的欧氏最小距离；无障碍图用地图对角长度哨兵，并计数。
不输出 CPU 快慢结论：共享主机未做安静测量，也没有板上延迟、CPU 节省或能耗测量。

`success_probe.py` 是独立的软件研究入口，保留旧模型、地图和验收；运行命令见方案文档。
它检查55个障碍起点失败，并测试精确目标优先的安全容差处理；不能把合成用例通过称为实车成功率提升。

新增安全目标容差和显式同步输入；旧API、旧数据与旧验收保留。新合成验证有安全覆盖提升，旧障碍起点不会被清空。

软件v3：NavFn单格起点处理、距离代价与受约束平滑，见[V3_README](v3_readme.md)。原图安全与新策略图安全分别报告。

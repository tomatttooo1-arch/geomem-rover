# 代价图与 Nav2 代价层逐格比对（376 帧）

## 比的是什么
融合核的代价图操作（GRID：把融合地图投影成导航平面，并按 Nav2 膨胀层的代价公式做膨胀）在宿主机上对照：
同一条实车录制轨迹（rec30，391 帧被融合接受）逐帧回放，其中 376 帧的高度带落在窗口内、产生导航平面（另 15 帧在合成边界段，高度带不在窗口内，两条路线都不出平面，跳过）。

- 对照台：`tools/b1_costmap/bench_int.py`；Nav2 参照：`tools/b1_costmap/nav2_ref.py`（按 Nav2 Humble 膨胀层的代价公式与参数实现：膨胀半径 0.45 m、代价缩放 3.0、机器人半径 0.30 m、分辨率 0.05 m）。
- 结果文件：`costmap_grid_bench.json`（对照台原样输出）。

## 结果（`costmap_grid_bench.json` 中的字段）

| 比较 | 帧数 | 结果 | 字段 |
|---|---|---|---|
| 代价图核输出 对 独立 C 模型 | 655 次 GRID 调用 | 1,423,200 个 64 位字 0 差异，报头 0 差异 | `grid_calls`、`grid_words_compared`、`grid_words_diff`、`grid_header_diff` |
| 对 Nano 上的软件桥节点（Nav2 代价层的输入栅格） | 376 | 栅格、严格三态、高位平面均 0 格差异 | `bridge_frames`、`bridge_*_cells_diff` |
| 膨胀代价 对 Nav2 膨胀公式精确参照（`nav2_ref.inflate_exact`：每格取所有致命格代价的最大值） | 376 | 0 格差异 | `exact_cost_frames`、`exact_cost_cells_diff` |

补充：Nav2 实际运行的是波前近似（`nav2_ref.inflate_nav2` 复刻）。在设计阶段同一轨迹的比对中，代价图核膨胀结果与波前复刻在共约 240.6 万格中有 12 格不同（最大差 2 个代价单位），这 12 格与"波前复刻 对 精确参照"的 12 格是同一批，来自 Nav2 波前算法自身的近似。

## 阳性对照（证明比对手段能看见错误）

| 对照 | 被抓到的帧 | 字段 |
|---|---|---|
| 故障构建：忽略 y 方向环形偏移 | 376 帧中 292 帧（其余帧 y 偏移为 0，本就不可见） | `controls/GRID_RING_Y_detected` |
| 故障构建：漏写最后一个辅助字 | 376 / 376 | `controls/GRID_LAST_AUX_detected` |
| 输入篡改：高度带下沿 +1 | 376 帧中 93 帧 | `controls/wrong_band_detected` |

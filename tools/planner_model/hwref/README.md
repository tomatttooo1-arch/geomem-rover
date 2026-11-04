# B 宿主参考与重放

交付头文件在上一级 `planner_hw_ref.hpp`，只用标准C++14头文件；不依赖ROS、Vitis、第三方库。`detail::Engine::scan`及其整数调用树是拟放PL的语义边界；完整`plan`含PS连续几何。没有HLS pragma，也没有综合/硬件性能证据。`Engine<400,400,...>`只供桥接器作宿主参考统计，公开入口仅80×80。

既有产物在本人副本 `runs/planner-hwref-20261103`。200向量（约5MB）不进Git；提交的 `vector_manifest_20261103.json` 保存每个文件的SHA256，可在项目内读取该产物目录。黄金模型固定21069ec；旧v3源文件保持逐字节不变，观察口仅加到runs里的复制件，并对全部2024请求核对状态、路径和resolved未变。

```bash
# 本人clone根目录；OUT必须是新目录，避免覆盖已有原始结果。
OUT=runs/planner-hwref-replay-new
cmake -S tools/planner_model -B "$OUT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/build" -j2
python3 -m tools.planner_model.hwref.verify --out "$OUT" --phase profile
# 核对推荐K；当前冻结头文件K=6。不要为新图失败改K凑回归。
python3 -m tools.planner_model.hwref.verify --out "$OUT" --phase verify
python3 -m tools.planner_model.hwref.direct_compare --out "$OUT"
python3 -m tools.planner_model.hwref.smoothing --out "$OUT"
"$OUT/build/planner_hwref_replay" "$OUT/vectors"
cmake -S tools/planner_model -B "$OUT/build" -DFSQ_HWREF_VECTORS="$PWD/$OUT/vectors"
(cd "$OUT/build" && ctest --output-on-failure)
```


向量目录编号000..199，每组六文件：

- `map.u8`：原始行序80×80地图，6400字节。
- `request.txt`：sx sy gx gy limit，一行；包含nan非法输入阴性对照，读取器用stod。
- `expected.txt`：status path_points field_valid rounds，一行。
- `field.u24le`：黄金势场，每格3字节低字节先，INF=FFFFFF；未产生完整场时空文件。K超限的部分场不是期望解，禁止使用。
- `path.f64le`：黄金完整第1档路径，交替x,y，IEEE binary64小端；K超限预期空路径。
- `changed.u8`：新半邻域的每轮标志，每字节0/1；正常完整场必为若干1后一个0，K超限全1。这是扫描协议，不要求与旧8邻版本的轮内更新位置相同。

180组地图向量在legacy/synthetic/fresh各取60个均匀输入序号；20组控制含全障碍、围起点、边界、切角、同起终点、连续目标、容差及K1/蛇形K6拒绝。期望字段/路径来自黄金，K拒绝来自明确的失败协议；不能把未产生场的文件说成黄金完整场。独立C++读取器比较全部有效场、路径每个字节、标志、状态和输入未改。

K=6是当前2024回归分布的选择：12张固定seed随机高代价障碍图均需要11–17轮，K=6全部安全拒绝；蛇形控制在原v3需要20轮。后续产品接入必须有PS回退，再在真实地图分布上评估K，不能直接把K=6当通用成功保证。

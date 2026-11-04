# 规划软件 v3

按用户要求统一 NavFn 单格清起点语义，修复距离代价和取路；这是宿主 CPU 软件，不是 HLS 或车上部署。

## 使用

```bash
cmake -S tools/planner_model -B runs/planner-v3-build -DCMAKE_BUILD_TYPE=Release
cmake --build runs/planner-v3-build -j2
(cd runs/planner-v3-build && ctest --output-on-failure)
python3 -m tools.planner_model.plan_v3_request \
  --map runs/planner-v2-delivery-20261102/aligned_example_costmap.npy \
  --request runs/planner-v2-delivery-20261102/aligned_example_request.json \
  --lib runs/planner-v3-build/libplanner_model.so \
  --out runs/planner-v3-example-result.json
```

输入 NPY+JSON 格式、SHA256 绑定、坐标系和同步要求沿用 [v2](v2_readme.md)；输出文件必须不存在。
Python 用 `PlannerV3(...).plan(...)`；返回 `start_policy=navfn_clear_cell`、原始起点代价及 `original_map_hits`，不能将原图命中误读成无障碍。
C++ ABI 见 [model_v3.hpp](model_v3.hpp)。固定点为 uint64、浮点为 float64，两者累加相同整数。

`plan_cells(..., variant=...)` 提供固定消融：

- 0：只清起点，保留等权代价和旧取路。
- 1：直步/斜步权重10/14，取路按前驱势能＋边代价选择，同代价优先靠近起点，再固定行序。
- 2（默认）：在1上做连续安全、最小离障不降低的最远可见点跳连；端点保持不变，不满足约束就保留原段。

内部复制地图且只将起点格置0，邻格和障碍/未知阈值不变；原始目标>=253仍拒绝，包括与被阻塞起点重合的目标。被包围起点仍失败。
检查实际世界位姿接线时也只允许同一个起点格例外，触及任何其他禁行格就拒绝。

## 复现与检查

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m tools.planner_model.validate_v3 \
  --baseline runs/planner-model-final-20261102 \
  --v2 runs/planner-v2-delivery-20261102 \
  --out runs/planner-v3-reproduce-20261102
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 -m unittest discover -s tests -p 'test_planner*.py' -v
```

使用 discovery 从本地 tests 目录加载，避免没有 `tests/__init__.py` 时误导入环境中的同名包。

完整2024对×3档×2种数值，含旧1449、新v2合成275及固定新增300。旧结果作为已见回归，不称新盲测；全部图/起终点不作事后修改。
长度统计在双方成功且NavFn长度>1e-12的请求上计算，零长度不参与长度比，但仍保留在成功率分母。
原图安全和单格清起点策略图安全分别报告，不能用后者声称旧原图安全门通过。


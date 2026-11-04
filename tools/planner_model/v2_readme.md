# 规划软件v2：同步输入与安全目标容差

这版是宿主CPU软件实现，不依赖ROS，不改变车上导航。C++14/CMake；Python3.8+、numpy、scipy，画图另需matplotlib。
代码：`model_v2.cpp/.hpp`；输入适配器：`aligned_v2.py`；文件请求：`plan_v2_request.py`。

## 复跑整套测试

从仓库根运行，`--baseline`使用上一轮已固定的同图/同起终点原始证据；输出目录必须新建或为空：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m tools.planner_model.validate_v2 --baseline runs/planner-model-final-20261102 --out runs/planner-v2-reproduce-20261102
```

该命令编译两个C++测试，执行30个容差边界对照、9个输入/安全控制、文件入口阳性/错误哈希阴性、1449对旧回归与275对新合成验证，保存完整请求、路径、统计和样例输入。
旧基线不存在时，先按本目录原README从formal.bin生成v1基线；勿用其他输入或随机采样结果冒充旧基线。
新集是预先固定的合成验证集，不是实车盲测。所有无效输入与不可达请求保留在统计里。

## 用自己的同步数据请求规划

地图为二维uint8 `.npy`，0–252可走，253–255拒绝。JSON请求必须包含：

| 字段 | 内容 |
|---|---|
| `map_sha256` | `.npy`文件原始字节的SHA256，绑定本张地图 |
| `metadata` | `frame_id,sequence,stamp_ns,resolution,origin_x,origin_y,origin_yaw,coordinates` |
| `pose` | `frame_id,sequence,stamp_ns,x,y,provenance`，与本张图严格匹配 |
| `goal` | `frame_id,x,y`，已转换到地图坐标系的世界位置 |

`provenance`为`recorded_pose`、`synthetic_pose`或`test_fixture`，不得把合成位姿称作真实录制。
`coordinates`必须显式选择：

- `cell_center`：标准栅格含半格中心偏移，世界位置先逆原点旋转平移，再floor取格。
- `navfn_round`：复刻固定Humble包装器的无半格偏移/round约定。

函数不执行TF查询、不猜窗口中心、不改图清障、不插值补位姿。上游应先把位姿变换并同步到本张图的帧/时间。
元数据一致不等于真实标定一定正确，仍须上游保证原点、坐标系、外参、时间和位姿的真实性。

整套验证会生成一个可直接使用的请求（`--out`不可覆盖已有文件）：

```bash
python3 -m tools.planner_model.plan_v2_request --map runs/planner-v2-delivery-20261102/aligned_example_costmap.npy --request runs/planner-v2-delivery-20261102/aligned_example_request.json --lib runs/planner-v2-delivery-20261102/build/libplanner_model.so --out runs/planner-v2-example-result.json
```

结果含`status,success,path,world_path,requested_goal_world,resolved_goal_world`以及帧号、时间和位姿来源。
退出码：0成功；1请求合法但无安全路径；2输入契约/文件/参数不合法。
核心状态：`exact`、`within_tolerance`、`invalid_start`、`invalid_goal`、`no_path`、`safety_rejected`等。
到达容差点仅表示导航条件满足，物品是否找到仍由到场观测确认。

## 算法与边界

势场从起点初始化0，正反扫描至收敛；保留NavFn `int(50+0.8*c)`单格代价、8邻同权、固定平局顺序。
精确目标优先；只有它不可达才用0.15m方形容差扫描，依次检查可达性、最近目标和终点连线安全。
取路反向后，C++检查完整线段；世界输入适配器还检查真实起点至代表格点的连线。致命/未知目标及障碍起点都拒绝。
float64与uint32分别计算；上限1024×1024保证有限势能小于UINT32_MAX/4哨兵；单次请求路径上界W×H+2。
当前每请求重建势场，没有跨请求缓存，不把工作量模型当作板上延迟。

v2保留旧`model.cpp`及NavFn源码。8邻同权仍会产生较长路径，本轮没有捆绑权重或平滑调参。
旧数据缺真实同步位姿，依然无法据此验证实车配准修复或称其成功率已提高。

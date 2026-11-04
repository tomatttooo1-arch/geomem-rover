# 真实深度＋事后目标回放（仅宿主 CPU）


**字段名不证明位姿来源。** 指定的rec30.pack是恒等wc，car570是ground-calib安装矩阵，car1015也是恒定安装矩阵。car570不能据此构造行驶轨迹；car1015从原始all.npz的c2w恢复世界位姿，并逐帧校验ids/stamps/K/深度。该轨迹是RTAB-Map离线关键帧优化加相对odom修正，不是实时TF。

在本人的独立工作树里运行，输出目录必须不存在。`E2`、`G1D`、`OLD`是只读输入；`R`是新的本人产物目录，不要覆盖旧趟或队友目录。

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PY=${FPGA_HOME}/miniconda3/bin/python3.13
R=runs/hindsight-new
$PY -m tools.planner_model.hindsight.pipeline audit --e2 "$E2" --g1d "$G1D" --out "$R/audit"
$PY -m tools.planner_model.hindsight.pipeline calibrate --e2 "$E2" --g1d "$G1D" --trace "$TRACE" --old-maps "$OLD" --out "$R/calibration"
$PY -m tools.planner_model.hindsight.controls --pack "$E2/rec30.pack" --out "$R/chain-positive"
# 校准非零就停。下一条也会验证calibration.json的passed。
$PY -m tools.planner_model.hindsight.pipeline car1015 --e2 "$E2" --g1d "$G1D" --calibration "$R/calibration/calibration.json" --out "$R/car1015"
cmake -S tools/planner_model -B "$R/planner-build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$R/planner-build" -j 2
(cd "$R/planner-build" && ctest --output-on-failure)
$PY -m tools.planner_model.hindsight.compare --root "$R" --lib "$R/planner-build/libplanner_model.so" --navfn-lib "$R/planner-build/libplanner_realpose_navfn.so"
```

校准源适配：rec30只用pack深度/内参，复现formal.bin原有合成odom、mount、FollowCameraGrid；核对所有310帧flags、融合header/delta和成本图。旧B1模拟体清不改。它证明C链路逐位复现，不能证明真实位姿。car1015用恢复的世界相机位姿重新生成几何块，再用车中心移窗与体清；参数不调。

`comparison/*_{pairs,paths,rejected,control_rows}.jsonl`保留每帧证据。`*_pose_controls.json`使用**体清前**的成本图，正常非致命>=95%，固定X+1m坏格率增加>=10个百分点；Y+1m和90°仅诊断。回环修正、时差、地面高度假设都要随报告保留。门不过仍能生成明确标为诊断的算法表，不能声称真实确认。差异图若某算法无路径，在图例明确写no path，未来轨迹不代表当时地图上可达。

两解释器专项：

```bash
# 同命令将解释器换为 /usr/bin/python3.8
```

不运行HLS、Vivado或设备；没有计时性能结论。不会改冻结算法、既有录制工具或B参考模型。


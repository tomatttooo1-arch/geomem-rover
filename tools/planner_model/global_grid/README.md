# 双尺寸规划核的宿主参考统计


固定原图身份、保守0.05→0.10m聚合/未知补边、选点方式在profile.py和预注册中。原参考头不改，直接实例化Engine<200,200,0x1fffffff>。每张仅一次完整建场，不计CPU时间，不扫K或开关。

现有原始结果：`runs/planner-global-20261104/profile/`，教室6轮、TUM fr2回放4轮。文件名中的known_footprint_m表示源PGM的矩形尺寸（其中包含未知格），不能当成全部已观测面积。生成来源和统计摘要已原样复制到docs/team/planner_global_20261104。

确有复现需要时，在本人工作树用一个全新输出目录执行：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python3 -m tools.planner_model.global_grid.profile --out runs/planner-global-profile-new
```


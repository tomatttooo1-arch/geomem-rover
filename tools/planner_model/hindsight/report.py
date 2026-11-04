"""Generate the hindsight delivery from immutable run outputs; never edit raw results."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
from tools.planner_model.hindsight.inputs import sha, write_json


def load(p):return json.loads(Path(p).read_text())
def num(x):return '—' if x is None else '%.4f'%x
def pct(x):return '—' if x is None else '%.2f%%'%(100*x)
def status(x):return '通过' if x else '未通过'


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--root',required=True)
    ap.add_argument('--doc',required=True);ap.add_argument('--assets',required=True)
    ap.add_argument('--doc-only',action='store_true',help='refresh narrative from existing immutable evidence assets')
    a=ap.parse_args();root=Path(a.root);doc=Path(a.doc);assets=Path(a.assets)
    if not a.doc_only:assets.mkdir(parents=True,exist_ok=False)
    c=load(root/'calibration'/'calibration.json');g=load(root/'car1015'/'generation.json')
    p=load(root/'chain-positive'/'control.json');s=load(root/'comparison'/'summary.json')
    selected=load(root/'comparison'/'selected_examples.json');audit=load(root/'audit'/'pack_audit.json')
    ds={r['dataset']:r for r in s['datasets']}
    proof={'calibration.json':root/'calibration'/'calibration.json','generation.json':root/'car1015'/'generation.json',
           'summary.json':root/'comparison'/'summary.json','pack_audit.json':root/'audit'/'pack_audit.json',
           'chain_positive.json':root/'chain-positive'/'control.json','selected_examples.json':root/'comparison'/'selected_examples.json'}
    if not a.doc_only:
        for n,src in proof.items():shutil.copyfile(src,assets/n)
        for r in selected:shutil.copyfile(root/'comparison'/r['image'],assets/r['image'])
        for name in ['rec30','car1015']:
            for suffix in ['pose_controls','body_seeded_controls']:
                src=root/'comparison'/(name+'_'+suffix+'.json');shutil.copyfile(src,assets/src.name)
    asset_rel=assets.name
    lines=['# 事后目标规划验证结果（dev2，2026-11-03）','',
      '**回放与诊断交付完成，真实位姿确认未通过。** 指定pack的world_camera字段并非逐帧世界位姿：car570缺可关联行驶轨迹；car1015已从原始NPZ恢复离线轨迹并执行固定比较。确认资格、样本量和算法门分开报告，未用模拟运动补真实样本。','',
      '独立工作树`work/${FPGA_SRC}`，分支`feat/dev2/planner-hindsight-20261103`。另一对话在`work/dev2`推进B，本任务未改其副本/分支，不重复B；只用宿主CPU，无HLS/Vivado、GPU、设备、main集成或部署。','',
      '## 预注册及输入来源','',
      '先提交预注册`d52012a`，发现来源误解后、尚未生成确认集地图/抽取目标时提交来源适配`c942fad`。两节只追加于[gates](../../gates/dev2_PLANNER_REALPOSE_20261102.md)。v3固定`21069ecedb8865e32ca801b6fc9244e5ffbc3835`、第2档默认并列第1档，参数不调。每5帧采样，第一个未来XY距离>=1m的点；若此前首次出有效窗[1,78]，取轨迹交点。未知/致命目标拒绝，起点仅内部清一格，完整原图命中另存。','',
      '| 数据 | pack帧数／不同矩阵数 | 实际位姿来源与使用资格 |','|---|---:|---|']
    descriptions={'rec30':'pack矩阵恒等；formal.bin原构造使用合成odom和推荐mount，只作回归',
       'car570':'37种矩阵是分批安装外参；据打包脚本说明来源为57批标定快照，缺逐帧世界轨迹',
       'car1015':'pack矩阵等于恒定mount；从元数据指向all.npz的c2w恢复离线世界轨迹，作为诊断确认候选'}
    for name in ['rec30','car570','car1015']:
        r=audit[name];lines.append('| %s | %d／%d | %s |'%(name,r['frames'],r['unique_matrices'],descriptions[name]))
    lines += ['',
      '直接检查打包源码`E2/bundle_ab84136/code/tools/gpu_geom/make_pack.py`和`G1c/tools/make_car1015_pack.py`：car的wc来自mount，后者每帧复用同一Tf，不能按字段名当世界坐标。car570平移X恒约0.1m、Y恒0，Z只是安装高度差；不把标定批次切换解释成开车。','',
      'car1015原始数据`D_gsplat/data/all.npz`保留c2w、ids、stamps、K；本轮逐项核对元数据身份、单调时间、pack内参和全部深度。源代码`extract_all.py`用175个优化关键帧，其他1054节点用“最近关键帧优化base×逆原始odom(kf)×原始odom(node)×mount”。因此它含事后优化及修正切换，不能称实时TF。安装外参是base←camera optical；本轮按`T_world_body=T_world_camera×inverse(T_body_camera)`换算，已用带90°转向的独立已知答案测试核对逆变换方向。','',
      '## C模型链路校准','',
      'rec30.pack只提供深度与内参；按formal.bin已有源码的`odom_base(k) @ base_camera()`和FollowCameraGrid重建原合成几何输入，再接滚动融合C模型和B1代价C模型。校准B1体清仍采用旧生成器的窗口中心抖动，半径/高度带/膨胀参数未改；这一来源适配在读确认地图前固定。原pack恒等矩阵未被冒称真实轨迹。','',
      '- 实际校准%d帧，成本数组逐字节比较%d字节；flags、几何块原点、滚动窗、融合header/delta、成本图差异计数均为0。校准%s；先通过才生成car1015地图。'%(c['compared_frames'],c['compared_bytes'],status(c['passed'])),
      '- 整链阳性：同一rec30首帧将全部深度代码置零，flags改变%d格；delta由%d条变为%d条；成本改变%d字节。参数、内参、位姿、窗口一致，证明链路可观察变化。'%(p['differing_flags'],p['normal_delta_words'],p['blank_delta_words'],p['differing_cost_bytes']),
      '- car1015全部%d帧逐像素深度对齐，恢复mm后与pack差异%d像素；使用恢复世界位姿的32×32×10前向块、80×80×32滚动窗、0.05m栅格，height_cell=8，窗口锚40/40/8，B1 ground_z=0、robot_height=0.31m、体清0.35m及原膨胀参数。'%(len(g['frames']),g['depth_mismatch_pixels']),
      '- car1015车中心XYZ范围%s至%s；相邻帧最大XY距离%s m。这是离线轨迹口径，含运动与回环修正，不能当控制延迟或实时TF质量。'%(g['translation_min'],g['translation_max'],num(g['maximum_step_xy'])),
      '', '## 位姿正常／阳性对照','',
      '预注册正常非致命(cost<253)比例>=95%，固定世界X+1m后的致命/未知/出界比例须增加>=10个百分点。用体清前的成本图检验；额外报告正式体清图，防止“先把车附近置自由，再证明车在自由格”的循环证据。Y+1m、绕世界原点90°只作诊断，不替代固定X对照。','',
      '| 数据／平面 | 正常非致命 | X+1m坏格 | 增加百分点 | 两对照同时成立 |','|---|---:|---:|---:|---|']
    for name in ['rec30','car1015']:
        for suffix,label in [('pose_controls','体清前（预注册）'),('body_seeded_controls','体清后（诊断）')]:
            r=load(root/'comparison'/(name+'_'+suffix+'.json'));st=r['stats'];n=st['normal']['n']
            lines.append('| %s／%s | %d/%d (%s) | %d/%d (%s) | %s | %s |'%(name,label,
                n-st['normal']['bad'],n,pct(st['normal']['nonfatal_rate']),st['x_plus_1m']['bad'],n,pct(st['x_plus_1m']['bad_rate']),
                '—' if r['x_positive_increase'] is None else '%.2f'%(100*r['x_positive_increase']),status(r['passed'])))
    bad_hist=load(root/'comparison'/'car1015_pose_controls.json')['stats']['normal']['cost_histogram']
    lines += ['', 'car1015体清前坏格组成：cost=253（膨胀内切区）%s帧、255（未知）%s帧、254（占据致命）%s帧；不能把所有坏格都称作真实障碍碰撞。'%tuple(bad_hist.get(k,0) for k in ['253','255','254']), '', 'car570没有世界轨迹，两个位姿对照均无有效分母，不算通过。体清前检查失败说明当前地图/轨迹组合未建立所需校准证据，不能仅凭这一失败断言安装外参方向错误；未知观测覆盖、地面高度和离线轨迹修正均需进一步区分。原位姿阈值未放宽。','',
       '## 样本与拒绝','', '| 数据 | 总采样 | 接受 | 拒绝 | 拒绝原因 |','|---|---:|---:|---:|---|']
    for name in ['car570','car1015','rec30']:
        r=ds[name];lines.append('| %s | %d | %d | %d | %s |'%(name,r['requests'],r['accepted'],r['rejected'],json.dumps(r['rejection_reasons'],ensure_ascii=False)))
    lines += ['', 'N=5使car570最多114个采样、rec30最多62个；不足200按实数报告。没有重复样本、缩短距离、换自由目标或合并三组补足。目标现在未知/致命时，即使未来车实际到过，也按旧时刻地图拒绝。','',
       '## 固定五项（每组单列）','',
       '安全门：只豁免请求起点一格，其余原图整条闭合栅格折线命中=0。覆盖门>=99%；双方成功且NavFn长度>1e-12m的长度比中位<=1.05、p95<=1.15；候选最小离障中位>=NavFn全部成功输出中位−1格；定浮完整路径一致>=99%。第1档为加权原始路径，第2档为默认平滑输出，不能把平滑成绩替代原始路径成绩。','']
    for name in ['car570','car1015','rec30']:
        r=ds[name];lines += ['### '+name,'']
        if r['variants'] is None:
            lines += ['两档五项均无可比较请求；安全/覆盖/长度/离障/定浮均无数据，未通过。全请求NavFn与候选成功数均为0/%d（输入被拒绝，不是算法不可达结论）。'%r['requests'],''];continue
        lines += ['| 指标 | 第1档 | 第2档（默认） |','|---|---:|---:|']
        v1,v2=r['variants']['1'],r['variants']['2']
        fields=[('安全：命中输出数',lambda v:str(v['policy_collision_outputs'])),
          ('NavFn成功／接受请求',lambda v:'%d/%d'%(v['navfn_success'],r['accepted'])),
          ('候选成功／总采样',lambda v:'%d/%d (%s)'%(v['candidate_success'],r['requests'],pct(v['all_request_success_rate']))),
          ('覆盖NavFn成功',lambda v:'%d/%d (%s)'%(v['covered_navfn_success'],v['navfn_success'],pct(v['coverage']))),
          ('长度比中位／p95 (n)',lambda v:'%s / %s (%d)'%(num(v['length_ratio']['median']),num(v['length_ratio']['p95']),v['length_ratio']['n'])),
          ('最小离障中位／NavFn（格）',lambda v:'%s / %s'%(num(v['clearance_candidate_median']),num(v['clearance_navfn_median']))),
          ('定浮完整路径相同',lambda v:'%d/%d (%s)'%(v['fixed_matches'],v['fixed_compared'],pct(v['fixed_rate']))),
          ('完全不豁免原图命中输出数',lambda v:str(v['original_collision_outputs'])),
          ('五项门',lambda v:'，'.join(k+':'+status(x) for k,x in v['gates'].items()))]
        for label,fn in fields:lines.append('| %s | %s | %s |'%(label,fn(v1),fn(v2)))
        lines += ['', 'NavFn全请求成功率%s；NavFn策略图有命中输出%d条。接受样本<200：%s；预注册位姿校准：%s；真实确认标志：%s。'%(pct(r['navfn_all_request_success']),r['navfn_policy_collision_outputs'],'是' if r['below_200'] else '否',status(r['pose_calibration']),r['real_pose_confirmation']), '']
    lines += ['## 五个差异例','',
      '按预注册排序：默认第2档与NavFn只有一方成功优先，其次第1档长度比偏离1的绝对值降序，dataset/frame打破平局。同图叠加当帧代价图、NavFn、第1/2档及对应未来观测轨迹；缺失路径在图例写no path。这些是局部规划诊断图，未来观测轨迹不是Nav2请求。','']
    for j,r in enumerate(selected):
        rr=r['row'];lines += ['%d. `%s` 帧%d：NavFn=%s，第1档=%s，第2档=%s，未来目标帧%d。'%(j+1,rr['dataset'],rr['frame'],rr['navfn']['success'],rr['v3_1']['success'],rr['v3_2']['success'],rr['future_frame']),
          '', '![差异例%d](%s/%s)'%(j+1,asset_rel,r['image']), '']
    if len(selected)<5:lines += ['可比较输入不足以产生5张路径差异图，仅提供实际%d张，不伪造路径。'%len(selected),'']
    lines += ['## 验证、证据及交接','',
       '- Python3.13和3.8受影响规划检查各41项通过，无跳过；含8项新的输入/目标/坐标/端到端计数及失败校准阻断检查。固定C++ CTest三组通过。冻结v3编译源码与21069ec逐文件无差异。',
       '- fpga-hls使用dev2配置`${FPGA_HOME}/dev1/.codex-accounts/dev2/skills/fpga-hls`；本轮实际隔离自测44/44，已知报告7.016ns、DSP43、LUT4536，缺报告返回3。这是skill校准，不是规划硬件综合或性能结果。',
       '- 所有失败趟保留：首次unittest模块入口碰到外部同名tests包，改用仓库discover；旧CTest不支持--test-dir，已在构建目录实际跑3组；新增fixture最初没随车移窗导致+1m出界，修正fixture后原断言通过。未调整实验判据或候选算法。',
       '- 原始产物`%s`，逐帧pairs/paths/rejected/control_rows、原始日志及各输入SHA256完整保留。仓内证据见[%s](%s/summary.json)、[校准](%s/calibration.json)、[清单](%s/MANIFEST.sha256)。'%(root.resolve(),'汇总',asset_rel,asset_rel,asset_rel),
       '- 复现入口[README](../../tools/planner_model/hindsight/README.md)。本队新增：pack来源审计、受门控的C链路适配、真实轨迹身份核对、事后目标与证据交付。厂商/开源：固定Humble NavFn及RTAB-Map来源轨迹；既有本队模块：M7几何/融合、B1成本图与v3。未使用U19/RISCV旧工程。','',
       '结论不外推：遥控未来位置不是实际Nav2目标分布；离线优化轨迹不是实时TF；旧时刻可规划性不由未来到达保证；本轮不证明实车安全、整机加速或时延收益。car570仍需对应世界轨迹或新录制，真实导航确认仍按A录制清单交owner现场执行。B由另一对话继续。']
    doc.write_text('\n'.join(lines)+'\n')
    if a.doc_only:
        print(doc)
        return
    # Manifest is generated from actual final files, never patched by hand.
    manifest=[]
    for pth in sorted(root.rglob('*')):
        if pth.is_file() and 'build' not in pth.parts and 'planner-build' not in pth.parts:
            manifest.append(sha(pth)+'  '+str(pth.resolve()))
    for pth in sorted(assets.iterdir()):
        if pth.is_file() and pth.name != 'MANIFEST.sha256':manifest.append(sha(pth)+'  '+str(pth.resolve()))
    (assets/'MANIFEST.sha256').write_text('\n'.join(manifest)+'\n')
    print(doc)


if __name__=='__main__':main()

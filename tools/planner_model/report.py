"""Render reviewed numeric results and original PNGs into the requested handoff document."""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil


def number(x):
    return '无样本' if x is None else ('%.4f' % x).rstrip('0').rstrip('.')


def percent(x):
    return '无样本' if x is None else '%.2f%%' % (100*x)


def dist(d):
    return '%s / %s / %s（n=%d）' % (number(d['median']),number(d['p95']),number(d['max']),d['n'])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True)
    ap.add_argument('--dest',default='docs/team/dev2_PLANNER_MODEL_RESULT_20261102.md')
    a=ap.parse_args();run=Path(a.run);dest=Path(a.dest);dest.parent.mkdir(parents=True,exist_ok=True)
    s=json.loads((run/'results/summary.json').read_text())
    gen=json.loads((run/'input/generation.json').read_text())
    rows=[json.loads(x) for x in (run/'results/pairs.jsonl').read_text().splitlines()]
    inv=s['frame_inventory'];hold=s['summaries']['holdout'];all_=s['summaries']['all']
    asset=dest.parent/'planner_model_20261102';asset.mkdir(exist_ok=True)
    for prefix in ['worst','quality']:
        for i in range(1,6):shutil.copyfile(run/('results/%s_%d.png'%(prefix,i)),asset/('%s_%d.png'%(prefix,i)))
    def table_summary(name,r):
        f=r['float'];sm=r['smooth']
        return '| %s | %d | %d/%d（%s） | %s / %s | %s / %s | %s |' % (
            name,r['pairs'],r['wave_success'],r['navfn_success'],percent(r['success_rate']),
            number(f['length_ratio']['median']),number(f['length_ratio']['p95']),
            number(sm['length_ratio']['median']),number(sm['length_ratio']['p95']),percent(r['fixed_match_rate']))
    lines=['# FPGA 规划核软件模型结果（dev2，2026-11-02）','',
           '**本次结论：不过线，不建议据此开写规划硬件。** 原始 8 邻同权波前的成功率和路径长度均未过预注册门槛；简单平滑虽改善长度，成功率仍未过。输入也不足以证明真实障碍场景下的价值。该结论仅针对任务指定的首版算法、给定模拟起点与这批输入；不等于所有快速扫描或硬件规划方案均不可行。没有修改车上导航、没有部署、没有集成 main。','',
           '## 1. 固定版本与可复现性','',
           '- 任务书：[dev1 原任务副本](dev2_TASK_PLANNER_MODEL_20261102.md)，门槛原样保留于 [预注册](../../gates/dev2_PLANNER_MODEL_20261102.md)。',
           '- 实验代码提交：`'+(run/'CODE_COMMIT.txt').read_text().strip()+'`；首轮 `9cac5fd` 产物保留在 `runs/planner-model-20261102/`，审计版在 `'+str(run)+'`。',
           '- 预注册在首轮评测前提交；本次没有调算法常数。第 1–100 张只作开发集，第 101–376 张是正式留出。前后两版算法、采样、输入、门槛未变；计量修复覆盖差集见 [audit.md](../../tools/planner_model/audit.md)。',
           '- 固定 Nav2 Humble 源码提交 `3c3db59d6969d8ecee8e68468693d006397f4a0c`，不是重新实现一个近似 NavFn。其 ROS 日志由空宏替换，核心文件未改；纯 C++14 / CMake、不依赖 ROS。当前车上二进制是否正好对应这个上游提交未确认。',
           '- 源输入与相邻元数据 SHA256：`'+gen['trace_sha256']+'` / `'+gen['metadata_sha256']+'`。生成图数组 SHA256：`'+gen['maps_sha256']+'`。',
           '- 所有数字为本会话直接运行所得；CPU 是共享主机，未做安静计时，故不报告速度提升、Nano CPU 节省、板上延迟或能耗。硬件拍数只是工作量模型。','',
           '一条命令（用新输出目录，原目录禁止覆盖）：','',
           '```bash',
           'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m tools.planner_model.run --trace ${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin --out runs/planner-model-reproduce-20261102',
           '```','',
           '需要 C++14、CMake、Python 3.8+、numpy/scipy/matplotlib。详见 [README](../../tools/planner_model/README.md)。','',
           '## 2. 输入核验与关键限制','',
           'B1 工具按 dev1 的 `4cfaed7` 原样复制到本分支。391 次成功 INTEGRATE 中，15 次高度带不在窗口内，留下 376 张 80×80、0.05 m 图。全部回放头部/增量与录包逐字相同；投影及车体种子与软件桥逐格相同；C 模型膨胀与精确参考逐格相同。孤立障碍轴向 0–13 格实测代价：`'+str(gen['isolated_axial'])+'`，符合任务书所列 5/6/7/8/9/10 格常数。','',
           'C 模型膨胀与 Nav2 波前复刻在整批图合计差 '+str(gen['cost_vs_nav2_differing_cells'])+' 格，最大差 '+str(gen['cost_vs_nav2_max_abs'])+'。这项差异保留；两种规划器都吃同一份 **C 模型 cost 平面**，没有给某一方不同地图。输入包含 0.35 m 未知种子，不额外引入 Nano swept 清除策略。','',
           '**376 张不全是 rec30。** 同轨迹含 310 张 rec30、43 张 TUM、23 张合成边界图。回放也没有真实车位姿；起点只能按 B1 的窗口中心＋确定性微扰取格 (40,40)。这个约定导致 rec30 起点全部被膨胀障碍覆盖，不能当成真实车位姿处的规划验收。','',
           '| 来源 | 图数 | 起点 cost≥253 | 没有可走目标的图数 | 实际查询对 | NavFn 成功 | 波前成功 |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for label in ['rec30','synthetic','tum']:
        ff=[f for f in inv if f['source']==label];rr=[r for r in rows if r['source']==label]
        lines.append('| %s | %d | %d | %d | %d | %d | %d |'%(label,len(ff),sum(f['start_cost']>=253 for f in ff),sum(f['goals']==0 for f in ff),len(rr),sum(r['navfn_success'] for r in rr),sum(r['float_success'] for r in rr)))
    lines += ['',
        '采样种子为 `RandomState(20261102 + 生成图序号)`；每图尽量取 20 个不同可走格，再取距边界≤3 格、离起点最远且未重复的至多 5 格。已预注册，正式结果后没有扩大“近边缘”范围或清空障碍来凑数。实际 '+str(sum(f['goals']==0 for f in inv))+' 图零目标，'+str(sum(f['edge_goals']==0 for f in inv))+' 图没有可用边缘目标。未达到名义 376×25=9400 对，只得到 '+str(all_['pairs'])+' 对；留出 276 图得到 '+str(hold['pairs'])+' 对。缺样本本身就是输入限制。',
        '', 'NavFn 依其原生封装清空自身起点并使用目标容差；波前按任务要求不从障碍起点出发。完整线段安全检查仍用原图，因此把 NavFn 返回了路径与安全通过分开报告。留出 NavFn 成功输出中，有 '+str(sum(bool(r['navfn_hits']) for r in rows if 100<r['frame']<=376 and r['navfn_success']))+' 条触及原图 cost≥253，合计 '+str(hold['navfn_safety_cells'])+' 次“每路径去重后的命中格”。这些对保留在正式分母，未事后删掉。','',
        '## 3. 第 5 节正式总表（101–376 张）','',
        'NavFn 成功分母 '+str(hold['navfn_success'])+' 对；波前 '+str(hold['wave_success'])+' 对成功。长度只在双方成功且 NavFn 长度非零的 '+str(hold['float']['length_ratio']['n'])+' 对上计算，失败由成功率独立惩罚。离障的 NavFn 中位数取所有 NavFn 成功对，候选中位数取其成功路径。','',
        '| 指标 | 原门槛 | 原始波前 | 简单视线平滑 | 判断（原始 / 平滑） |',
        '|---|---|---|---|---|']
    f=hold['float'];sm=hold['smooth']
    def judgement(key):return ('过' if f['gates'][key] else '不过')+' / '+('过' if sm['gates'][key] else '不过')
    lines += [
        '| 安全 | 致命/未知穿越=0 | %d 格次 | %d 格次 | %s |'%(f['safety_cells'],sm['safety_cells'],judgement('safety')),
        '| 成功率 | ≥99%% | %d/%d=%s | 同原始 | %s |'%(hold['wave_success'],hold['navfn_success'],percent(hold['success_rate']),judgement('success')),
        '| 长度比中位 / p95 | ≤1.05 / ≤1.15 | %s / %s | %s / %s | %s |'%(number(f['length_ratio']['median']),number(f['length_ratio']['p95']),number(sm['length_ratio']['median']),number(sm['length_ratio']['p95']),judgement('length')),
        '| 离障最小距离的中位（格） | ≥NavFn−1 | %s（NavFn %s） | %s（NavFn %s） | %s |'%(number(f['clearance_wave_median']),number(f['clearance_navfn_median']),number(sm['clearance_wave_median']),number(sm['clearance_navfn_median']),judgement('clearance')),
        '| 定点/浮点原始路径全同 | ≥99% | '+percent(hold['fixed_match_rate'])+'（915/915；含共同失败） | 平滑输入相同 | '+judgement('fixed')+' |','',
        '成功路径单独看，定点/浮点全同为 860/860；全部本地图/大图输出的波前原始与平滑路径也都经线段安全检查，命中数均为 0。安全检查包括未知、两顶点之间的跨越及角接触，不是仅查路径顶点。','',
        '不同统计范围（长度为原始与 NavFn 比值，平滑单列；没有据此改变验收集）：','',
        '| 范围 | 查询数 | 波前/NavFn成功 | 原始长度中位/p95 | 平滑长度中位/p95 | 定/浮全同 |',
        '|---|---:|---|---|---|---|']
    for key,label in [('holdout','正式留出'),('all','全部376图'),('development','前100图'),('rec30','全部rec30'),('rec30_holdout','留出rec30'),('exact_goal_holdout','留出NavFn精确到目标'),('safe_navfn_holdout','留出NavFn安全输出（解释用）'),('large','三张合成大图（补充）')]:
        lines.append(table_summary(label,s['summaries'][key]))
    lines += ['',
        '留出 rec30 子集没有可采样查询，所以其长度/成功率无样本，不能称通过。可比较的波前成功本地路径均来自 TUM 的已知车体圆盘，周围主要是未知边界；未形成真实房间障碍物的主测试覆盖。即便只看安全 NavFn 输出的解释用子集，原始长度仍不过线。','',
        '## 4. 扫描轮数与硬件工作量估计','',
        '每个目标单独建势场，轮数含最后一轮无变化检查；被起点检查提前拒绝的查询没有扫图（rounds=0），不混进收敛分布。n 是实际建场目标数，不是图帧数。p95 使用 numpy 线性分位，因此可为小数。','',
        '| 输入 | 轮数 中位 / p95 / 最大 | 扫描拍数 中位 / p95 / 最大 |',
        '|---|---|---|',
        '| 80×80 全部本地输入 | '+dist(all_['rounds'])+' | '+dist(all_['cycles'])+' |',
        '| 80×80 留出 | '+dist(hold['rounds'])+' | '+dist(hold['cycles'])+' |']
    for i in range(1,4):
        rr=s['large_per_map'][str(i)]
        lines.append('| 合成400×400图%d | %s | %s |'%(i,dist(rr['rounds']),dist(rr['cycles'])))
    lines += ['| 合成大图合计 | '+dist(s['summaries']['large']['rounds'])+' | '+dist(s['summaries']['large']['cycles'])+' |','',
        '估算为 `轮数 × W × H × 2`，假设每格每遍 1 拍；8 邻读取、依赖、外存、初始化、路径提取、平滑和通信均未综合验证，不能把拍数当板上实测延迟。扫描优化如何映射到硬件仍未做。','',
        '未找到本副本可用的约400×400 RTAB-Map快照，补充使用明确合成的房间/门/家具、交替折返走廊、家具岛三图；各 20 个随机目标。外围膨胀使“距边界≤3格”没有可走目标，未补造额外5个。大图原始路径长度不过线；平滑虽过长度，离障中位仅 '+number(s['summaries']['large']['smooth']['clearance_wave_median'])+' 格，对照 '+number(s['summaries']['large']['smooth']['clearance_navfn_median'])+' 格，未过离障门槛。','',
        '## 5. NavFn失败而波前成功的逐例审计','',
        '本地图这类查询为 0。大图有 7 对，均在折返走廊（图2）；没有放宽障碍/未知阈值，波前路径逐条连续安全检查均为 0 命中。记录如下，诊断不改变正式成功分母：','',
        '| 图/对编号（对从0计） | 目标格 | NavFn阶段 | 默认提取步数 | 放大提取限额后的诊断步数 | 波前命中格 |',
        '|---|---|---|---:|---:|---:|']
    exceptions=[r for r in rows if not r['navfn_success'] and r['float_success']]
    for r in exceptions:
        d=r['navfn_diagnostics']
        lines.append('| %d/%d | %s | %d | %d | %d | %d |'%(r['frame']-1000,r['pair'],r['goal'],d[0],d[2],d[6],len(r['float_hits'])))
    lines += ['',
        '阶段 3＝没找到可用势能，4＝找到了势能但 calcPath 提取失败，5＝成功。若“放大提取限额”诊断产生路径，只能归因为默认 `4×max(W,H)` 路径提取预算限制；该诊断结果不替代任务要求的真实 NavFn 默认行为。','',
        '## 6. 三项阳性对照','',
        '1. 开放图已规划路径中间画墙：浮点、定点和 NavFn 都重新绕开，连续检查 0 命中；把修改前旧路径注入新地图，检查器抓到 '+str(len(s['controls']['injected_wall_hits']))+' 个墙格。',
        '2. 目标置254、另测255：两方入口均失败。额外调用未经目标拒绝的原生 NavFn：本例确实通过容差返回了附近位置。因此“致命目标拒绝”是任务要求的公共入口限制，不冒充 NavFn 原生规则。',
        '3. 在固定4对开放图斜线查询中去掉斜向：原始长度比中位 '+number(s['controls']['diagonal_control']['normal_median'])+' → '+number(s['controls']['diagonal_control']['mutant_median'])+'，变坏 '+number(s['controls']['diagonal_control']['mutant_median']-s['controls']['diagonal_control']['normal_median'])+'；证明长度计量能检测算法变坏。正式留出对应中位 '+number(f['length_ratio']['median'])+' → '+number(hold['mutant']['length_ratio']['median'])+'、p95 '+number(f['length_ratio']['p95'])+' → '+number(hold['mutant']['length_ratio']['p95'])+'。','',
        'C++ CTest 1项通过；独立几何/采样专项 Python 3.13、3.8 各7项通过。未运行无关全量测试。','',
        '## 7. 失败和差异最大案例 PNG','',
        '代价图黑色为障碍/未知，白色为自由；蓝点起点、紫星目标；青线 NavFn、红线原始波前、绿线平滑。失败时没有伪造波前线。','']
    for prefix,title,examples in [('worst','失败优先的最差5例',s['worst']),('quality','双方成功时长度差最大的5例',s['quality'])]:
        lines += ['### '+title,'']
        for i,r in enumerate(examples,1):
            ratio = r['float_length']/r['navfn_length'] if r['float_success'] and r['navfn_length'] else None
            lines += ['- 图%d，%s，对%d，起点%s（cost=%d），目标%s；长度比%s，NavFn命中%d格，波前%s。'%(r['frame'],r['source'],r['pair'],r['start'],r['start_cost'],r['goal'],number(ratio),len(r['navfn_hits']),'成功' if r['float_success'] else '拒绝障碍起点'),'',
                      '![图%d 对%d](planner_model_20261102/%s_%d.png)'%(r['frame'],r['pair'],prefix,i),'']
    lines += ['## 8. 常数、坐标、贡献边界与交接','',
        '- 单格代价 `int(50 + 0.8*c)`；依据 [NavFn Humble navfn.hpp / setCostmap](https://github.com/ros-navigation/navigation2/tree/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_navfn_planner)。先截断再累加，与原生ROS代价转换相同，未调参。0–252可走，≥253不可走。uint32势能；400²格简单路径上界 `(160000−1)×251 = 40159749`，16位不够。float64与整数累加同一份截断代价，不引入两种代价定义。',
        '- 固定8邻同权、每格当前代价加邻居势能、固定行优先邻居序、每步选最低势能，禁止斜向穿障碍角。没有加 sqrt(2) 权重，也没有改成 NavFn 的连续 Eikonal 二次插值；两者算法差异正是本次需要评估的内容。',
        '- 坐标遵循固定Humble源码：worldToMap为round、mapToWorld没有半格偏移，查询位姿取索引×0.05m。几何检查用以整数为中心的闭合格方块，两方同坐标。NavFn保留反向建场/反向提取及smoothApproachToGoal末端处理。',
        '- 简单平滑是最远可见路径点跳连，只删中间点，通过同一个连续安全检查；不把其结果混入原始路径指标。离障是完整折线到最近cost≥253（含未知和膨胀障碍）格中心距离，1格=5cm。',
        '',
        '| 清单 | 内容与来源 |','|---|---|',
        '| 厂商/开源 | 固定 Nav2 Humble 核心 BSD 源码、ROS封装参考Apache-2.0；numpy/scipy/matplotlib为评测依赖。Vitis Vision distanceTransform仅作思路来源，没有复制其代码。 |',
        '| 本队旧代码复用 | dev1的B1 C模型/膨胀复刻/bench、现有滚动融合CPU核/软件桥/回放解析器。U19与RISCV复用为0。 |',
        '| 本队新增（dev2） | 独立float64/uint32波前、无ROS对照入口、连续几何计量、确定性采样、阳性对照、统计/来源核验/PNG/可复现命令与本报告。 |','',
        '交接建议：本轮停在软件验证，保持车上Nav2。若dev1/owner希望继续此方向，应先提供真实车位姿对齐、至少能形成有代表性可走起终点的图集，再另行预注册算法变更（例如斜向距离权重/插值取路/代价约束平滑）。本次没有用留出集继续调参，没有回改第5节门槛。','',
        '原始证据位于 `'+str(run)+'`：`input/generation.json`逐图清单，`results/{summary.json,pairs.jsonl,paths.jsonl,controls.json,navfn_fail_wave_success.json}`及大图输入与PNG；正式结果的SHA256清单随分支提交。']
    dest.write_text('\n'.join(lines)+'\n')
    print(dest)


if __name__=='__main__':main()

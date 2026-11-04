"""End-to-end observable positive control: blank depth must change flags, delta and costs."""
import argparse
from pathlib import Path
import numpy as np
from host.geo_producer import DEFAULT_HEIGHT_CELL
from host.m7_grid_policy import FollowCameraGrid
from tools.car_geo_producer.common import odom_base, base_camera
from tools.planner_model.hindsight.inputs import Pack, write_json
from tools.planner_model.hindsight.pipeline import Chain
from tools.b1_costmap import bench as b


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pack',required=True);ap.add_argument('--out',required=True)
    a=ap.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    chain=Chain(out/'build');rec=Pack(a.pack).frame(0)
    tb=odom_base(0);wc=tb@base_camera()
    grid=FollowCameraGrid(.05,(32,32,10),back_cells=1,height_cell=DEFAULT_HEIGHT_CELL).grid_for(wc)
    base=(-40,-40,-8);ob=tuple(int(round(v/.05)) for v in grid.origin_m)
    results=[]
    for epoch,blank in [(1,False),(2,True)]:
        r=dict(rec)
        if blank:r['raw']=np.zeros_like(rec['raw'])
        flags,counts=chain.flags(r,wc,grid.origin_m,grid.shape)
        h,*_=chain.roll.call(0,epoch,0,base,(0,0,0),(0,0,0));assert int(h[0])==0
        h,delta,*_=chain.roll.call(1,epoch,1,base,ob,grid.shape,obs=flags.tobytes());assert int(h[0])==0
        costs=chain.costs(base,b.body_pose(base,1))
        results.append((flags,delta,costs,counts))
    normal,mutant=results
    result=dict(control='all depth codes zero, same first rec30 frame K/mount/synthetic pose/window/B1 parameters',
        differing_flags=int(np.count_nonzero(normal[0]!=mutant[0])),
        normal_delta_words=len(normal[1]),blank_delta_words=len(mutant[1]),
        differing_cost_bytes=int(np.count_nonzero(normal[2]!=mutant[2])),
        normal_counts=normal[3],blank_counts=mutant[3])
    result['passed']=result['differing_flags']>0 and result['normal_delta_words']>0 and result['blank_delta_words']==0 and result['differing_cost_bytes']>0
    np.savez_compressed(out/'normal_vs_blank.npz',normal_cost=normal[2],blank_cost=mutant[2])
    write_json(out/'control.json',result);print(result)
    if not result['passed']:raise SystemExit('chain positive control failed')


if __name__=='__main__':main()

"""Quality and forced-candidate diagnostics on preregistered representative frames."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def quality(rgb,depth):
    gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
    points=cv2.goodFeaturesToTrack(gray,1000,.01,3)
    xy=np.empty((0,2),int) if points is None else np.rint(points[:,0,:]).astype(int)
    occupied={(min(7,int(x)*8//gray.shape[1]),min(5,int(y)*6//gray.shape[0])) for x,y in xy}
    return {'laplacian_variance':float(cv2.Laplacian(gray,cv2.CV_64F).var()),
        'dark_fraction':float(np.mean(gray<=5)),'bright_fraction':float(np.mean(gray>=250)),
        'valid_depth_fraction':float(np.mean(depth>0)),
        'gftt_capped_count':len(xy),'gftt_occupied_cells_of_48':len(occupied),
        'feature_valid_depth_fraction':float(np.mean(depth[xy[:,1],xy[:,0]]>0)) if len(xy) else None}


def main():
    p=argparse.ArgumentParser();p.add_argument('selection',type=Path);p.add_argument('pairs',type=Path)
    p.add_argument('input',type=Path);p.add_argument('output',type=Path);p.add_argument('--plot',type=Path)
    a=p.parse_args();cv2.setNumThreads(1)
    selection=json.loads(a.selection.read_text());pairs=[json.loads(l) for l in (a.pairs/'pairs.jsonl').open()]
    controls={r['kind']:r['accepted'] for r in pairs if r['index']==-1}
    assert controls=={'positive_self':True,'positive_neighbor':True,'negative_black':False,'negative_no_depth_both':False}
    rows=[];images={}
    for item in selection['selected']:
        index=item['index'];t=item['timestamp'];files={k:a.input/k/(t+'.png') for k in ['rgb_sync','depth_sync']}
        for k,f in files.items():assert hashlib.sha256(f.read_bytes()).hexdigest()==item['source_pair'][k]['input_sha256']
        rgb=cv2.imread(str(files['rgb_sync']));depth=cv2.imread(str(files['depth_sync']),cv2.IMREAD_UNCHANGED)
        candidates=[r for r in pairs if r['index']==index and r['kind']==item['category']]
        assert len(candidates)==39 and len({r['prior_id'] for r in candidates})==39
        accepted=[r for r in candidates if r['accepted']]
        best=max(candidates,key=lambda r:(r['accepted'],r['inliers'],r['matches'],-r['prior_id']))
        factors=[]
        for blur in (0,1):
            for zero in (0,1):
                image=cv2.GaussianBlur(rgb,(31,31),8.) if blur else rgb
                d=np.zeros_like(depth) if zero else depth
                factors.append({'blur':bool(blur),'zero_depth':bool(zero),'quality':quality(image,d)})
        assert factors[1]['quality']['valid_depth_fraction']==0
        assert factors[2]['quality']['laplacian_variance']<factors[0]['quality']['laplacian_variance']
        row={'index':index,'category':item['category'],'frozen_candidate':item['candidate'],
             'quality':quality(rgb,depth),'accepted_prior_ids':[r['prior_id'] for r in accepted],
             'best_pair':best,'frozen_candidate_pair':next(r for r in candidates if r['prior_id']==item['candidate']),
             'quality_factors':factors,'registration_factors':[r for r in pairs if r['index']==index and r['kind']=='quality_factor']}
        rows.append(row);images[index]=rgb
    # A zero-feature control proves that a saturated 1000-point metric is not a
    # claim that input quality had no effect.
    black=quality(np.zeros((480,640,3),np.uint8),np.zeros((480,640),np.uint16))
    assert black['gftt_capped_count']==0 and black['laplacian_variance']==0
    result={'selection_protocol':selection['protocol'],'population_counts':selection['population_counts'],
        'controls':controls,'quality_black_control':black,'opencv_quality_version':cv2.__version__,
        'summary':{c:{'selected':sum(r['category']==c for r in rows),
            'any_forced_candidate_pass':sum(r['category']==c and bool(r['accepted_prior_ids']) for r in rows)} for c in selection['population_counts']},
        'rows':rows,'limits':['forced-candidate raw-image registration bypasses online retrieval and may reextract different features from stored-word matching',
            'a successful pair demonstrates verifiable image overlap under this diagnostic, not absolute map localization accuracy',
            'all 39 failures do not establish absent map coverage; viewing geometry, feature robustness and input quality may remain confounded',
            'GFTT quality proxy is capped at 1000 with fixed quality=.01/minDistance=3, not a copy of internal RTAB feature statistics'],
        'source_sha256':{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [a.selection,a.pairs/'pairs.jsonl']}}
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    if a.plot:
        if a.plot.exists():raise ValueError('plot exists')
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(6,6,figsize=(16,12),layout='constrained')
        for col,category in enumerate(selection['population_counts']):
            for i,row in enumerate(r for r in rows if r['category']==category):
                left,right=axes[i,2*col:2*col+2];prior_id=row['best_pair']['prior_id']
                left.imshow(cv2.cvtColor(images[row['index']],cv2.COLOR_BGR2RGB));left.set_title(f"{category}\ninput {row['index']} / depth {row['quality']['valid_depth_fraction']:.0%}",fontsize=8)
                ref=cv2.imread(str(a.pairs/'prior-images'/f'{prior_id}.png'))
                right.imshow(cv2.cvtColor(ref,cv2.COLOR_BGR2RGB));right.set_title(f"prior {prior_id}: {'PASS' if row['best_pair']['accepted'] else 'FAIL'}\ninliers {row['best_pair']['inliers']} / matches {row['best_pair']['matches']}",fontsize=8)
                left.axis('off');right.axis('off')
        fig.suptitle('Fixed representative inputs and strongest tested prior image — not an absolute-pose evaluation',fontsize=12)
        fig.savefig(a.plot,dpi=150);plt.close(fig)
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()

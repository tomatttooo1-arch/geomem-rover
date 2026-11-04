#!/usr/bin/env python3
"""Score ROS probe using a fixed prior-map alignment, never refit current poses."""
import argparse
import json
from pathlib import Path
import numpy as np


def evaluate(events, truth, alignment):
    for kind in ('publish_begin','odom','odom_info','info','localization_pose'):
        stamps=[e['stamp_ns'] for e in events if e['event']==kind]
        if len(stamps)!=len(set(stamps)):
            raise ValueError('duplicate timestamp in '+kind)
    pubs = {e['stamp_ns']: e for e in events if e['event'] == 'publish_begin'}
    info = {e['stamp_ns']: e for e in events if e['event'] == 'info'}
    odom = {e['stamp_ns']: e for e in events if e['event'] == 'odom_info'}
    odom_poses = {e['stamp_ns']:e for e in events if e['event']=='odom'}
    def valid_odom_message(e):
        cov=np.asarray(e.get('covariance',[]),dtype=float)
        return bool(not e.get('lost',True) and cov.size==36 and np.isfinite(cov).all()
                    and max(cov[[0,7,14,21,28,35]]) < 9999.)
    def valid_actual_odom(stamp):
        e=odom_poses.get(stamp,{})
        if e.get('header',{}).get('frame_id')!='odom' or e.get('child_frame_id')!='camera_link': return False
        pose=e.get('pose',{}).get('pose',{})
        q=pose.get('orientation',{});p=pose.get('position',{})
        xyz=np.array([p.get(k,float('nan')) for k in ('x','y','z')],dtype=float)
        quat=np.array([q.get(k,float('nan')) for k in ('x','y','z','w')],dtype=float)
        cov=np.asarray(e.get('pose',{}).get('covariance',[]),dtype=float)
        return bool(np.isfinite(xyz).all() and np.isfinite(quat).all() and abs(np.linalg.norm(quat)-1)<.001
                    and cov.size==36 and np.isfinite(cov).all() and min(cov[[0,7,14,21,28,35]])>=0
                    and max(cov[[0,7,14,21,28,35]])<9999.)
    local = [e for e in events if e['event'] == 'localization_pose']
    rotation = np.asarray(alignment['alignment_rotation'])
    translation = np.asarray(alignment['alignment_translation_m'])
    if not (rotation.shape==(3,3) and translation.shape==(3,) and np.isfinite(rotation).all()
            and np.isfinite(translation).all() and np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6)
            and abs(np.linalg.det(rotation)-1)<1e-6):
        raise ValueError('invalid fixed SE3 alignment')
    scored = []
    for e in local:
        stamp = e['stamp_ns']
        p = e['pose']['pose']['position']
        xyz = np.array([p['x'],p['y'],p['z']],dtype=float)
        timestamp = stamp/1e9
        nearest = int(np.argmin(np.abs(truth[:,0]-timestamp)))
        dt = float(abs(truth[nearest,0]-timestamp))
        evidence = info.get(stamp, {})
        valid = bool(np.isfinite(xyz).all() and dt < .02 and stamp in pubs
                     and e['header']['frame_id'] == 'map')
        error = float(np.linalg.norm(rotation@xyz+translation-truth[nearest,1:4])) if valid else None
        linked = evidence.get('loop_closure_id',0)>0 or evidence.get('proximity_detection_id',0)>0
        q=e['pose']['pose'].get('orientation',{})
        quat=np.array([q.get(k,float('nan')) for k in ('x','y','z','w')],dtype=float)
        cov=np.asarray(e['pose'].get('covariance',[]),dtype=float)
        pose_usable=bool(np.isfinite(quat).all() and abs(np.linalg.norm(quat)-1)<.01
            and cov.size==36 and np.isfinite(cov).all() and min(cov[[0,7,14,21,28,35]])>=0
            and max(cov[[0,7,14,21,28,35]])<9999)
        scored.append({'index': pubs.get(stamp,{}).get('index'), 'stamp_ns':stamp,
                       'error_m':error,'truth_dt_s':dt,'map_match':linked,
                       'loop_id':evidence.get('loop_closure_id'),
                       'proximity_id':evidence.get('proximity_detection_id'),
                       'pose_usable':pose_usable,
                       'good': bool(valid and linked and pose_usable and valid_odom_message(odom.get(stamp,{}))
                                    and valid_actual_odom(stamp) and error <= .15)})
    scored.sort(key=lambda r:r['stamp_ns'])
    def first_streak(rows):
        streak=[]
        for r in rows:
            streak = streak+[r] if r['good'] else []
            if len(streak)==3: return [x['index'] for x in streak]
        return None
    def region(predicate):
        stamps=[s for s,p in pubs.items() if predicate(p)]
        return {'published':len(stamps),'feedback':sum(s in odom for s in stamps),
                'lost':sum(bool(odom.get(s,{}).get('lost',False)) for s in stamps),
                'finite_not_lost_below_invalid_covariance':sum(valid_odom_message(odom.get(s,{})) for s in stamps),
                'valid_odom_pose_and_info':sum(valid_actual_odom(s) and valid_odom_message(odom.get(s,{})) for s in stamps),
                'missing':[pubs[s]['index'] for s in stamps if s not in odom]}
    start = next((e for e in events if e['event']=='start'),{})
    blank_stop=start.get('options',{}).get('blank_stop')
    recovery = None
    if blank_stop is not None:
        end=int(blank_stop)
        valid_odom=[p['index'] for s,p in pubs.items() if end<=p['index']<end+30
                    and s in odom and valid_odom_message(odom[s]) and valid_actual_odom(s)]
        streak=first_streak([r for r in scored if r['index'] is not None and end<=r['index']<end+30])
        recovery={'window':[end,end+30],'first_not_lost_index':min(valid_odom) if valid_odom else None,
                  'three_good_localizations':streak,'meets_probe_condition':bool(valid_odom and streak)}
    complete=[e for e in events if e['event']=='complete']
    errors=[r['error_m'] for r in scored if r['error_m'] is not None]
    return {'protocol':'fixed prior map SE3 alignment; no current-run fitting; translation only',
            'all_frames':region(lambda p:True),'blank_frames':region(lambda p:p['blank']),
            'nonblank_frames':region(lambda p:not p['blank']),
            'localization_outputs':len(local),'distinct_localization_stamps':len({e['stamp_ns'] for e in local}),
            'first_three_good_localizations':first_streak(scored),
            'localization_rmse_m':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            'localization_max_error_m':max(errors) if errors else None,
            'recovery':recovery,'scored_localizations':scored,
            'complete_records':complete,
            'tf_edges':sorted({t['header']['frame_id']+'->'+t['child_frame_id']
                              for e in events if e['event']=='tf' for t in e['transforms']}),
            'limitations':'Same recording replay; valid odometry flag is not pose accuracy; missing samples remain visible.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('observations',type=Path)
    p.add_argument('truth',type=Path)
    p.add_argument('prior_alignment',type=Path)
    a=p.parse_args()
    events=[json.loads(line) for line in a.observations.read_text().splitlines()]
    print(json.dumps(evaluate(events,np.loadtxt(a.truth),json.loads(a.prior_alignment.read_text())),indent=2,allow_nan=False))

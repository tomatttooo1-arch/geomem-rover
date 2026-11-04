"""ROS-independent timestamp, rigid-transform and local-window matching rules."""
from bisect import bisect_left
from collections import deque
import math
import numpy as np


class MissingInput(ValueError):
    pass


def quaternion_matrix(q):
    q=np.asarray(q,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise MissingInput('invalid_quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def slerp(a,b,t):
    a=np.asarray(a,float);b=np.asarray(b,float)
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    dot=float(a.dot(b))
    if dot<0:b=-b;dot=-dot
    if dot>.9995:
        q=(1-t)*a+t*b;return q/np.linalg.norm(q)
    angle=math.acos(np.clip(dot,-1.,1.))
    return (math.sin((1-t)*angle)*a+math.sin(t*angle)*b)/math.sin(angle)


def matrix(translation,rotation):
    m=np.eye(4);m[:3,:3]=quaternion_matrix(rotation);m[:3,3]=translation
    if not np.isfinite(m).all():raise MissingInput('nonfinite_transform')
    return m


class Transforms:
    """Recorded TF tree. No extrapolation; bounded bracket interpolation only."""
    def __init__(self,records,max_bracket_ns=100000000):
        self.edges={};self.max_bracket_ns=max_bracket_ns
        parents={}
        for r in records:
            if r['parent']==r['child'] or not r['parent'] or not r['child']:
                raise MissingInput('invalid_tf_frame')
            matrix(r['translation'],r['rotation'])
            if r['child'] in parents and parents[r['child']]!=r['parent']:
                raise MissingInput('tf_parent_changed_within_epoch')
            parents[r['child']]=r['parent']
            self.edges.setdefault((r['parent'],r['child']),[]).append(r)
        for key,rr in self.edges.items():
            # Dynamic and static authority for the same edge is ambiguous.
            if len(set(bool(r['static']) for r in rr))!=1:raise MissingInput('mixed_static_dynamic_tf')
            rr.sort(key=lambda r:r['stamp_ns'])
            for a,b in zip(rr[:-1],rr[1:]):
                if a['stamp_ns']==b['stamp_ns'] and not np.allclose(matrix(a['translation'],a['rotation']),matrix(b['translation'],b['rotation']),atol=1e-10,rtol=0):
                    raise MissingInput('conflicting_tf_at_same_stamp')
            if rr[0]['static']:
                first=matrix(rr[0]['translation'],rr[0]['rotation'])
                if any(not np.allclose(matrix(r['translation'],r['rotation']),first,atol=1e-10,rtol=0) for r in rr):
                    raise MissingInput('changing_static_tf')

    def at(self,rr,t):
        if rr[0]['static']:
            r=rr[0];return matrix(r['translation'],r['rotation']),[r['stamp_ns']]
        ts=[r['stamp_ns'] for r in rr];j=bisect_left(ts,t)
        if j<len(rr) and ts[j]==t:
            r=rr[j];return matrix(r['translation'],r['rotation']),[t]
        if j==0 or j==len(rr):raise MissingInput('tf_extrapolation')
        a,b=rr[j-1],rr[j];lo,hi=a['stamp_ns'],b['stamp_ns']
        if t-lo>self.max_bracket_ns or hi-t>self.max_bracket_ns:
            raise MissingInput('tf_bracket_too_wide')
        u=(t-lo)/(hi-lo)
        return matrix((1-u)*np.array(a['translation'])+u*np.array(b['translation']),
                      slerp(a['rotation'],b['rotation'],u)),[lo,hi]

    def lookup(self,target,source,t):
        if target==source:return np.eye(4),[]
        graph={}
        for (parent,child),rr in self.edges.items():
            try:m,stamps=self.at(rr,t)
            except MissingInput:continue
            evidence=dict(parent=parent,child=child,static=bool(rr[0]['static']),stamps_ns=stamps)
            graph.setdefault(child,[]).append((parent,m,evidence))
            graph.setdefault(parent,[]).append((child,np.linalg.inv(m),evidence))
        queue=deque([(source,np.eye(4),[])]);seen={source}
        while queue:
            current,m,used=queue.popleft()
            for nxt,edge,evidence in graph.get(current,[]):
                if nxt in seen:continue
                result=edge.dot(m);provenance=used+[evidence]
                if nxt==target:return result,provenance
                seen.add(nxt);queue.append((nxt,result,provenance))
        raise MissingInput('missing_or_stale_tf_chain')


def transform_xy(transform,points):
    points=np.asarray(points,float).reshape(-1,2)
    if not np.isfinite(points).all():raise MissingInput('nonfinite_coordinates')
    return np.column_stack((points,np.zeros(len(points)),np.ones(len(points)))).dot(transform.T)[:,:2]


def to_raw(grid,points):
    p=np.asarray(points,float).reshape(-1,2)-np.asarray(grid['origin'][:2])
    c,s=math.cos(grid['origin'][2]),math.sin(grid['origin'][2])
    return p.dot(np.array([[c,-s],[s,c]]))/grid['resolution']


def from_raw(grid,points):
    p=np.asarray(points,float).reshape(-1,2)*grid['resolution']
    c,s=math.cos(grid['origin'][2]),math.sin(grid['origin'][2])
    return p.dot(np.array([[c,s],[-s,c]]))+np.asarray(grid['origin'][:2])


def inside(point,bounds):
    low,high=bounds
    return bool(np.all(point>=low) and np.all(point<=high))


def first_exit(points,low,high):
    """First inside-to-outside segment intersection with a closed rectangle."""
    points=np.asarray(points,float).reshape(-1,2);low=np.asarray(low);high=np.asarray(high)
    if len(points)<2 or not inside(points[0],(low,high)):raise MissingInput('path_not_starting_inside_window')
    for a,b in zip(points[:-1],points[1:]):
        if inside(b,(low,high)):continue
        d=b-a;candidates=[]
        for axis in [0,1]:
            if d[axis]>0:candidates.append((high[axis]-a[axis])/d[axis])
            elif d[axis]<0:candidates.append((low[axis]-a[axis])/d[axis])
        valid=[u for u in candidates if -1e-12<=u<=1+1e-12 and inside(np.clip(a+u*d,low,high),(low,high))]
        if not valid:raise MissingInput('no_window_exit')
        u=min(valid);return a+u*d
    raise MissingInput('global_path_does_not_exit_window')


def select_map(maps,request_stamp,max_age_ns=200000000):
    before=[m for m in maps if m['stamp_ns']<=request_stamp]
    if not before:raise MissingInput('no_past_map')
    chosen=max(before,key=lambda m:m['stamp_ns'])
    if request_stamp-chosen['stamp_ns']>max_age_ns:raise MissingInput('stale_map')
    if chosen['width']!=80 or chosen['height']!=80 or abs(chosen['resolution']-.05)>1e-6:
        raise MissingInput('unsupported_map_geometry')
    return chosen

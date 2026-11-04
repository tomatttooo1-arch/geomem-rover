"""Read rosbag2 directly using Humble rosbag2_py; emit ROS-independent events."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def stamp(message):
    return int(message.sec)*1000000000+int(message.nanosec)


def pose_origin(p):
    q=p.orientation
    # Costmap origins must be planar; never silently discard roll/pitch.
    if abs(q.x)>1e-6 or abs(q.y)>1e-6:raise ValueError('nonplanar_map_origin')
    yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
    return [float(p.position.x),float(p.position.y),yaw]


def normalized(topic,kind,msg,received,params=None):
    if kind=='tf2_msgs/msg/TFMessage':
        for t in msg.transforms:
            yield dict(kind='tf',stamp_ns=stamp(t.header.stamp),parent=t.header.frame_id,
                child=t.child_frame_id,translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                rotation=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w],static=topic=='/tf_static')
    elif kind=='nav2_msgs/msg/Costmap':
        info=msg.metadata
        yield dict(kind='map',stamp_ns=stamp(msg.header.stamp),frame_id=msg.header.frame_id,
            width=int(info.size_x),height=int(info.size_y),resolution=float(info.resolution),
            origin=pose_origin(info.origin),costs=list(msg.data),encoding='nav2_uint8_raw')
    elif kind=='nav_msgs/msg/OccupancyGrid':
        if params is None:raise ValueError('occupancy_grid_requires_explicit_inflation_params')
        from tools.b1_costmap.nav2_ref import InflationParams, master_from_grid, inflate_exact
        info=msg.info;data=np.asarray(msg.data,dtype=np.int16).reshape(info.height,info.width)
        if np.any(data < -1) or np.any(data>100):raise ValueError('invalid_occupancy_values')
        costs=inflate_exact(master_from_grid(data),InflationParams(**params)).astype(np.uint8)
        yield dict(kind='map',stamp_ns=stamp(msg.header.stamp),frame_id=msg.header.frame_id,
            width=int(info.width),height=int(info.height),resolution=float(info.resolution),
            origin=pose_origin(info.origin),costs=costs.ravel().tolist(),encoding='reconstructed_trinary_inflated',inflation_params=params)
    elif kind=='geometry_msgs/msg/PoseStamped':
        yield dict(kind='request',stamp_ns=stamp(msg.header.stamp),frame_id=msg.header.frame_id,
            goal=[msg.pose.position.x,msg.pose.position.y],source='goal_pose_topic',request_id='%s:%d'%(topic,received))
    elif kind=='std_msgs/msg/String':
        r=json.loads(msg.data)
        for key in ['request_id','stamp_ns','frame_id','goal']:r[key]
        yield dict(r,kind='request',source='explicit_request_capture')
    elif kind=='nav_msgs/msg/Path':
        if not msg.poses:return
        if any(p.header.frame_id and p.header.frame_id!=msg.header.frame_id for p in msg.poses):
            raise ValueError('mixed_path_frames')
        yield dict(kind='plan',stamp_ns=stamp(msg.header.stamp),frame_id=msg.header.frame_id,
            points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])


def extract(uri,out,map_topic,request_topic,plan_topic,source_kind,params=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out=Path(out)
    if out.exists():raise ValueError('output_must_not_exist')
    out.mkdir(parents=True)
    source=Path(uri).resolve()
    manifest=[]
    for p in sorted(source.rglob('*')) if source.is_dir() else [source]:
        if p.is_file():manifest.append(dict(path=str(p),size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    (out/'input_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    storage='mcap' if (source.is_file() and source.suffix=='.mcap') or (source.is_dir() and any(source.glob('*.mcap'))) else 'sqlite3'
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source),storage_id=storage),rosbag2_py.ConverterOptions('cdr','cdr'))
    topic_types={t.name:t.type for t in reader.get_all_topics_and_types()}
    selected=[t for t in [map_topic,request_topic,plan_topic,'/tf','/tf_static','/clock'] if t in topic_types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    counts=Counter();errors=[];epoch=0;last_clock=None;last_tf={};static_cache={}
    with (out/'events.jsonl').open('w') as f:
        while reader.has_next():
            topic,data,received=reader.read_next();kind=topic_types[topic]
            counts[topic]+=1
            try:
                msg=deserialize_message(data,get_message(kind))
                if topic=='/clock':
                    now=stamp(msg.clock)
                    if last_clock is not None and now<last_clock:
                        epoch+=1;last_tf={}
                        for row in static_cache.values():f.write(json.dumps(dict(row,epoch=epoch))+'\n')
                    last_clock=now;continue
                records=list(normalized(topic,kind,msg,received,params))
                # Without /clock, a >1s backward jump on a TF edge starts a new epoch.
                for row in records:
                    if row['kind']=='tf' and not row['static']:
                        key=(row['parent'],row['child'])
                        if last_clock is None and key in last_tf and row['stamp_ns']<last_tf[key]-1000000000:
                            epoch+=1;last_tf={}
                            for old in static_cache.values():f.write(json.dumps(dict(old,epoch=epoch))+'\n')
                        last_tf[key]=row['stamp_ns']
                    if row['kind']=='tf' and row['static']:static_cache[(row['parent'],row['child'])]=row
                    row.update(epoch=epoch,topic=topic,received_ns=int(received))
                    if row['kind']!='tf' and row['stamp_ns']<=0:raise ValueError('missing_source_timestamp')
                    f.write(json.dumps(row,allow_nan=False)+'\n')
            except (ValueError,TypeError,KeyError) as error:
                errors.append(dict(topic=topic,received_ns=int(received),reason=str(error)))
    result=dict(source_kind=source_kind,bag=str(source),topics=topic_types,selected_topics=selected,
                counts=dict(counts),decode_errors=errors,epochs=epoch+1,map_topic=map_topic,
                request_topic=request_topic,plan_topic=plan_topic)
    (out/'extraction.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',required=True);ap.add_argument('--out',required=True)
    ap.add_argument('--map-topic',default='/local_costmap/costmap_raw')
    ap.add_argument('--request-topic',default='/dev2/planner_request')
    ap.add_argument('--plan-topic',default='/plan')
    ap.add_argument('--source-kind',required=True,choices=['real','simulation','test_fixture'])
    ap.add_argument('--inflation-params',help='JSON object; required for raw OccupancyGrid reconstruction')
    a=ap.parse_args();params=json.loads(Path(a.inflation_params).read_text()) if a.inflation_params else None
    print(json.dumps(extract(a.bag,a.out,a.map_topic,a.request_topic,a.plan_topic,a.source_kind,params),indent=2))


if __name__=='__main__':main()

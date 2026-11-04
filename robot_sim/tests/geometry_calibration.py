"""Analytic calibration of rendered architectural planes, never object inference."""
import math
import xml.etree.ElementTree as ET


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(matrix, vector):
    return [sum(matrix[i][j]*vector[j] for j in range(3)) for i in range(3)]


def rpy_matrix(roll, pitch, yaw):
    sr, cr=math.sin(roll), math.cos(roll)
    sp, cp=math.sin(pitch), math.cos(pitch)
    sy, cy=math.sin(yaw), math.cos(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def quaternion_matrix(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def ray_box(origin, direction, lower, upper):
    """Nearest ray parameter, keeping direction unnormalized (optical Z)."""
    near=-math.inf; far=math.inf; face=None
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if not lower[axis] <= origin[axis] <= upper[axis]:
                return None
            continue
        first=(lower[axis]-origin[axis])/direction[axis]
        second=(upper[axis]-origin[axis])/direction[axis]
        if first>second:first,second=second,first
        if first>near:near=first;face=axis
        far=min(far,second)
        if near>far:return None
    if near <= 0 or far <= 0:return None
    return near,face


def plane_samples(world_path, k, base_pose):
    k = [float(value) for value in k]
    world=ET.parse(world_path).find("world")
    sensor=world.find(".//sensor[@type='rgbd_camera']")
    pose=list(map(float,sensor.findtext("pose").split()))
    base_rotation=quaternion_matrix(base_pose.orientation)
    offset=apply(base_rotation,pose[:3])
    origin=[base_pose.position.x+offset[0],base_pose.position.y+offset[1],base_pose.position.z+offset[2]]
    optical_to_link=[[0,0,1],[-1,0,0],[0,-1,0]]
    rotation=multiply(multiply(base_rotation,rpy_matrix(*pose[3:])),optical_to_link)
    architecture=world.find("model[@name='architecture']/link")
    boxes=[]
    for visual in architecture.findall("visual"):
        xyzrpy=list(map(float,visual.findtext("pose").split()))
        if any(abs(x)>1e-12 for x in xyzrpy[3:]):
            raise ValueError("Calibration expects axis-aligned architectural boxes")
        size=list(map(float,visual.findtext("geometry/box/size").split()))
        boxes.append((visual.attrib['name'],[xyzrpy[i]-size[i]/2 for i in range(3)],
                      [xyzrpy[i]+size[i]/2 for i in range(3)]))
    result=[]
    # Fixed before measuring depth: tabletop and a different, vertical wall plane.
    for u,v,expected_visual,expected_axis in [(200,90,'table_visual',2),(280,40,'east_visual',0)]:
        direction=apply(rotation,[(u-k[2])/k[0],(v-k[5])/k[4],1.0])
        hits=[]
        for name,low,high in boxes:
            hit=ray_box(origin,direction,low,high)
            if hit is not None:hits.append((hit[0],name,hit[1]))
        if not hits:raise ValueError("Calibration ray misses known architecture")
        depth,visual,axis=min(hits)
        if visual!=expected_visual or axis!=expected_axis:
            raise ValueError(f"Calibration ray {(u,v)} hits {visual} axis {axis}, expected {expected_visual} axis {expected_axis}")
        result.append({'pixel':[u,v],'visual':visual,'plane_axis':axis,
                       'expected_optical_z_m':depth,'ray_origin_world':origin,
                       'ray_direction_world_per_optical_z':direction})
    return result

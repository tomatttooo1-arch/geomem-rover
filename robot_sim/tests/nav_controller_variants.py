#!/usr/bin/env python3
"""Controlled DWB/RPP candidate configs; formal default remains unchanged."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source-revision',required=True)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('Refusing to overwrite a controller probe run')
    args.output.mkdir(parents=True)
    source=Path(__file__).resolve().parents[1]/'fpga_sim/config/nav2_overrides.yaml'
    base=yaml.safe_load(source.read_text())
    rpp=deepcopy(base)
    rpp['controller_server']['ros__parameters']['FollowPath']={
        'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel':0.25,'lookahead_dist':0.6,
        'min_lookahead_dist':0.3,'max_lookahead_dist':0.9,'lookahead_time':1.5,
        'use_velocity_scaled_lookahead_dist':False,
        'rotate_to_heading_angular_vel':0.6,'max_angular_accel':1.0,
        'use_rotate_to_heading':True,'rotate_to_heading_min_angle':0.5,
        'transform_tolerance':0.2,'use_collision_detection':True,
        'max_allowed_time_to_collision_up_to_carrot':1.7,
        'use_regulated_linear_velocity_scaling':True,
        'use_cost_regulated_linear_velocity_scaling':True,
        'regulated_linear_scaling_min_radius':0.9,'regulated_linear_scaling_min_speed':0.1,
        'min_approach_linear_velocity':0.05,'approach_velocity_scaling_dist':0.6,
        'cost_scaling_dist':0.3,'cost_scaling_gain':1.0,'inflation_cost_scaling_factor':3.0,
    }
    # The mature controller differs; all outside-controller safety constraints stay identical.
    check=deepcopy(rpp)
    check['controller_server']['ros__parameters']['FollowPath']=base['controller_server']['ros__parameters']['FollowPath']
    if check!=base:raise RuntimeError('Non-controller behavior changed in candidate')
    cases=[]
    damped=deepcopy(rpp)
    # Conservative braking prediction for delayed reference-pose feedback.
    # Physical limits, velocity smoother, goal checker and timeouts are unchanged.
    damped['controller_server']['ros__parameters']['FollowPath']['max_angular_accel']=0.2
    heading=deepcopy(damped)
    # Single-factor candidate: retain XY/stopped-velocity checks and all budgets.
    heading['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']=0.005
    dynamic=deepcopy(heading)
    # Keep zero-command collision handling alive across the 1 Hz replanning cycle.
    dynamic['controller_server']['ros__parameters']['failure_tolerance']=2.0
    for name,config in [('dwb',base),('rpp',rpp),('rpp_damped',damped),('rpp_heading_precision',heading),
                        ('rpp_dynamic',dynamic)]:
        path=args.output/(name+'.yaml');path.write_text(yaml.safe_dump(config,sort_keys=False))
        cases.append({'name':name,'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    manifest={'source_revision':args.source_revision,'baseline_config_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'cases':cases,'same_depth_obstacle_layer':True,'same_progress_goal_checkers':False,
              'goal_checker_exception':{'case':'rpp_heading_precision','baseline':'rpp_damped',
                  'path':'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance',
                  'baseline_value':damped['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],
                  'candidate_value':0.005},
              'controller_patience_exception':{'case':'rpp_dynamic','baseline':'rpp_heading_precision',
                  'path':'controller_server.ros__parameters.failure_tolerance',
                  'baseline_value':'inherited from installed Nav2 parameters',
                  'candidate_value':2.0},
              'same_smoother_and_model_speed_acceleration_limits':True,
              'config_only_manifest':True,'current_task_budgets_defined_by_runtime':True,
              'legacy_reference_probe_conditions':{
                  'goal':{'x':0.0,'y':1.2,'yaw':0.0},'start':'fresh world initial pose',
                  'wall_timeout_seconds':90,'record_sim_elapsed_separately':True},
              'upstream':'https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller'}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Run one specialty episode: odom/lidar policy, independent Gazebo scoring."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def write(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tasks',required=True)
    parser.add_argument('--task',required=True)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--policy',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--run-id',required=True)
    opts=parser.parse_args()
    report={'schema_version':1,'task_id':opts.task,'status':'starting','passed':False,
            'started_unix_s':time.time(),'run_id':opts.run_id,'policy_file':opts.policy,'manifest':opts.manifest,
            'policy_observations':['commanded odometry relative to known spawn','lidar','relative waypoint','yaw error'],
            'ground_truth_role':'scoring only; never fed to policy','trajectory':[],
            'limits':{'max_translation_m_s':.25,'max_yaw_rad_s':.6},
            'raw_intersection_samples':0,'padded_intersection_samples':0,
            'geometry_samples':0,'min_raw_clearance_m':None,'min_padded_clearance_m':None,
            'path_length_m':0.,'yaw_distance_rad':0.,'max_fixed_heading_error_rad':0.,
            'max_cross_track_m':0.,'max_rotation_drift_m':0.,'combined_motion_sim_s':0.,
            'max_path_deviation_m':0.,
            'max_tracked_yaw_error_rad':0.,'telemetry_pause_wall_s':0.,'telemetry_pause_count':0,
            'yaw_reference_definition':'active commanded waypoint, matching frozen offline evaluator',
            'watchdog':{'pause_wall_age_s':1.5,'abort_wall_age_s':12.0},
            'odom_truth_error_max_m':0.,'valid_scan_frames':0,'all_inf_scan_frames':0,
            'command_trace':[],'waypoints_reached':0,'stop_verified':False}
    write(opts.output,report)
    node=None;rclpy=None;exit_code=1
    try:
        tasks=json.loads(Path(opts.tasks).read_text())
        candidates=tasks.get('evaluation_tasks',tasks.get('tasks',[]))
        task=next(t for t in candidates if t['id']==opts.task)
        report['task_spec']=task
        manifest=json.loads(Path(opts.manifest).read_text())
        loaded=json.loads(Path(opts.policy).read_text())
        report['policy_type']=loaded.get('type');report['policy_parameters']=loaded.get('params',loaded.get('parameters'))
        from policy import HolonomicPolicy
        policy=HolonomicPolicy(report['policy_parameters'])
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from ament_index_python.packages import get_package_share_directory
        sys.path.insert(0,str(Path(get_package_share_directory('qianli_training_scenarios'))/'scripts'))
        from benchmark import (prepare_obstacles,offset_polygon,transform_polygon,footprint_clearance)
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from rosgraph_msgs.msg import Clock
        raw=manifest['footprint'];padded=offset_polygon(raw,manifest['parameters']['footprint_padding'])
        obstacles=prepare_obstacles(manifest)
        start=task['start'];waypoints=task['waypoints']
        reference=np.asarray([start[:2]]+[p[:2] for p in waypoints],dtype=float)
        segments=reference[1:]-reference[:-1]
        segment_squared=np.sum(segments*segments,axis=1)
        thresholds=task.get('thresholds',{})
        required=set(task.get('required_constraints',[]))
        max_sim=task.get('max_sim_time_s',60.)
        kind=task.get('kind',task.get('ability',''))

        class Episode(Node):
            def __init__(self):
                super().__init__('qianli_omni_episode',parameter_overrides=[Parameter('use_sim_time',value=True)])
                self.time=None;self.odom=None;self.truth=None;self.scan=None
                self.odom_wall=self.truth_wall=self.scan_wall=0.
                self.clock_wall=0.;self.stream_stamps={};self.target_index=0
                self.last_truth=None;self.last_geometry=None;self.last_trace=-1.
                self.active=False;self.hit=False;self.estimated=None;self.valid_sequence=0
                self.last_cmd=np.zeros(3);self.last_command_time=None
                self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
                self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
                self.create_subscription(Odometry,'/simulation/ground_truth',self.on_truth,qos_profile_sensor_data)
                self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
                self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
            def on_clock(self,m):
                self.time=m.clock.sec+m.clock.nanosec*1e-9;self.clock_wall=time.monotonic()
                self.stream_stamps['clock']=self.time
            def on_odom(self,m):
                p=m.pose.pose;c,s=math.cos(start[2]),math.sin(start[2])
                self.odom=(p.position.x,p.position.y,yaw(p.orientation));self.odom_wall=time.monotonic()
                self.stream_stamps['odom']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                self.estimated=(start[0]+c*p.position.x-s*p.position.y,
                                start[1]+s*p.position.x+c*p.position.y,wrap(start[2]+self.odom[2]))
            def on_scan(self,m):
                self.scan=m;self.scan_wall=time.monotonic()
                self.stream_stamps['scan']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                count=int(np.isfinite(np.asarray(m.ranges)).sum())
                if count:
                    self.valid_sequence+=1;report['valid_scan_frames']+=1
                else:
                    self.valid_sequence=0;report['all_inf_scan_frames']+=1
            def on_truth(self,m):
                p=m.pose.pose;pose=(p.position.x,p.position.y,yaw(p.orientation))
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                self.truth=pose;self.truth_wall=time.monotonic()
                self.stream_stamps['truth']=stamp
                if not self.active:return
                if self.last_truth is not None:
                    previous,previous_time=self.last_truth
                    step=math.hypot(pose[0]-previous[0],pose[1]-previous[1])
                    rotation=abs(wrap(pose[2]-previous[2]))
                    report['path_length_m']+=step;report['yaw_distance_rad']+=rotation
                    dt=stamp-previous_time
                    if dt>0 and step/dt>=.04 and rotation/dt>=.08:
                        report['combined_motion_sim_s']+=dt
                self.last_truth=(pose,stamp)
                report['max_fixed_heading_error_rad']=max(report['max_fixed_heading_error_rad'],abs(wrap(pose[2]-start[2])))
                target_yaw=waypoints[min(self.target_index,len(waypoints)-1)][2]
                report['max_tracked_yaw_error_rad']=max(report['max_tracked_yaw_error_rad'],abs(wrap(target_yaw-pose[2])))
                report['max_rotation_drift_m']=max(report['max_rotation_drift_m'],math.hypot(pose[0]-start[0],pose[1]-start[1]))
                # Perpendicular drift relative to initial robot X; pure-lateral
                # tests move along +/-bodyY and require bodyX to remain fixed.
                dx,dy=pose[0]-start[0],pose[1]-start[1]
                cross=abs(math.cos(start[2])*dx+math.sin(start[2])*dy)
                report['max_cross_track_m']=max(report['max_cross_track_m'],cross)
                fractions=np.clip(np.sum((np.asarray(pose[:2])-reference[:-1])*segments,axis=1)/np.maximum(segment_squared,1e-12),0,1)
                deviation=float(np.min(np.linalg.norm(np.asarray(pose[:2])-reference[:-1]-fractions[:,None]*segments,axis=1)))
                report['max_path_deviation_m']=max(report['max_path_deviation_m'],deviation)
                if self.estimated:
                    report['odom_truth_error_max_m']=max(report['odom_truth_error_max_m'],math.hypot(pose[0]-self.estimated[0],pose[1]-self.estimated[1]))
                if stamp-self.last_trace>=.1:
                    report['trajectory'].append({'sim_time_s':stamp,'pose':list(pose),'estimated_pose':list(self.estimated) if self.estimated else None})
                    self.last_trace=stamp
                if self.last_geometry is not None and stamp-self.last_geometry<.05:return
                self.last_geometry=stamp
                for name,footprint in [('raw',raw),('padded',padded)]:
                    clearance,_,hits=footprint_clearance(transform_polygon(footprint,pose),obstacles)
                    key='min_'+name+'_clearance_m'
                    if report[key] is None or clearance<report[key]:report[key]=clearance
                    report[name+'_intersection_samples']+=bool(hits)
                    if hits:self.hit=True
                report['geometry_samples']+=1
            def send(self,cmd):
                m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_footprint'
                m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=map(float,cmd)
                self.pub.publish(m);self.last_cmd=cmd
            def telemetry(self):
                now=time.monotonic()
                return {name:now-getattr(self,name+'_wall') for name in ['clock','odom','truth','scan']}

        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        def interrupt(_signum,_frame):raise InterruptedError('Termination signal')
        signal.signal(signal.SIGTERM,interrupt)
        node=Episode()
        deadline=time.monotonic()+80
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if (node.time is not None and node.time>0 and node.estimated and node.truth and
                    node.valid_sequence>=4 and node.pub.get_subscription_count()>0):break
        else:raise TimeoutError('Episode needs clock, controller, odom, truth and four valid lidar frames')
        if math.hypot(node.odom[0],node.odom[1])>.05 or abs(node.odom[2])>.05:
            raise RuntimeError('Fresh Gazebo episode required: odom is not at zero')
        spawn_error=math.hypot(node.truth[0]-start[0],node.truth[1]-start[1])
        if spawn_error>.03 or abs(wrap(node.truth[2]-start[2]))>.03:
            raise RuntimeError('Gazebo spawn does not match declared task start')
        report['spawn_error_m']=spawn_error
        report['initial_physical_pose']=list(node.truth);report['initial_estimated_pose']=list(node.estimated)
        started=node.time;wall_started=time.monotonic();last_control=node.time;last_checkpoint=time.monotonic()
        target_index=0;node.active=True;report['status']='running';write(opts.output,report)
        paused_wall=None
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            if node.hit:report['terminal_reason']='footprint_intersection';break
            if node.time-started>max_sim:report['terminal_reason']='simulation_timeout';break
            if time.monotonic()-wall_started>max(150,max_sim*4):report['terminal_reason']='wall_timeout';break
            ages=node.telemetry()
            if max(ages.values())>1.5:
                node.send(np.zeros(3))
                if paused_wall is None:
                    paused_wall=time.monotonic();report['telemetry_pause_count']+=1
                if max(ages.values())>12.:
                    report['terminal_reason']='stale_telemetry'
                    report['terminal_telemetry_age_s']=ages
                    report['terminal_stream_stamps']=dict(node.stream_stamps)
                    break
                continue
            if paused_wall is not None:
                report['telemetry_pause_wall_s']+=time.monotonic()-paused_wall
                paused_wall=None;last_control=node.time
            if node.valid_sequence==0 and node.scan and np.isfinite(node.scan.ranges).sum()==0:
                # A single empty frame can occur in open space; bound sustained
                # emptiness by the timestamp and per-frame counts in the report.
                if report['all_inf_scan_frames']>10:report['terminal_reason']='invalid_lidar';break
            dt=node.time-last_control
            if dt<.05:continue
            last_control=node.time
            gx,gy,gh=waypoints[target_index];x,y,h=node.estimated
            dx,dy=gx-x,gy-y;c,s=math.cos(h),math.sin(h)
            angular=wrap((start[2] if task['yaw_mode']=='fixed' else gh)-h)
            if math.hypot(dx,dy)<=.12 and abs(angular)<=.08:
                target_index+=1;report['waypoints_reached']=target_index
                node.target_index=target_index
                if target_index==len(waypoints):report['terminal_reason']='completed';break
                gx,gy,gh=waypoints[target_index];dx,dy=gx-x,gy-y
                angular=wrap((start[2] if task['yaw_mode']=='fixed' else gh)-h)
            ranges=np.asarray(node.scan.ranges,dtype=float)[::5]
            ranges=np.clip(np.nan_to_num(ranges,nan=4.,posinf=4.,neginf=4.),.01,4.)
            angles=(node.scan.angle_min+np.arange(len(node.scan.ranges))*node.scan.angle_increment)[::5]
            cmd=np.asarray(policy.action([c*dx+s*dy,-s*dx+c*dy],angular,ranges,angles,min(dt,.15)),dtype=float)
            if cmd.shape!=(3,) or not np.isfinite(cmd).all():raise ValueError('Policy returned invalid action')
            speed=float(np.linalg.norm(cmd[:2]))
            if speed>.25:cmd[:2]*=.25/speed
            cmd[2]=np.clip(cmd[2],-.6,.6)
            node.send(cmd)
            if len(report['command_trace'])==0 or node.time-report['command_trace'][-1]['sim_time_s']>=.25:
                report['command_trace'].append({'sim_time_s':node.time,'command':cmd.tolist(),'target_index':target_index})
            if time.monotonic()-last_checkpoint>2:
                report['elapsed_sim_s']=node.time-started;report['current_physical_pose']=list(node.truth)
                write(opts.output,report);last_checkpoint=time.monotonic()
        # Settle after zero command; scoring includes the braking interval.
        stop_time=node.time;stop_wall=time.monotonic();stop_pose=None
        while node.time-stop_time<.7 and time.monotonic()-stop_wall<5:
            node.send(np.zeros(3));rclpy.spin_once(node,timeout_sec=.02)
            if node.time-stop_time>.45 and stop_pose is None:stop_pose=node.truth
        final=node.truth;final_goal=waypoints[-1]
        report['stop_verified']=bool(stop_pose and math.hypot(final[0]-stop_pose[0],final[1]-stop_pose[1])<.01 and abs(wrap(final[2]-stop_pose[2]))<.01)
        report['elapsed_sim_s']=node.time-started;report['elapsed_wall_s']=time.monotonic()-wall_started
        report['final_physical_pose']=list(final);report['final_estimated_pose']=list(node.estimated)
        report['final_position_error_m']=math.hypot(final[0]-final_goal[0],final[1]-final_goal[1])
        report['final_yaw_error_rad']=abs(wrap(final[2]-final_goal[2]))
        checks={'all_waypoints_completed':report['terminal_reason']=='completed',
                'within_sim_timeout':stop_time-started<=max_sim,
                'physical_final_position':report['final_position_error_m']<=thresholds.get('final_position_m',.15),
                'physical_final_yaw':report['final_yaw_error_rad']<=thresholds.get('final_yaw_rad',.12),
                'zero_raw_intersections':report['geometry_samples']>0 and report['raw_intersection_samples']==0,
                'zero_padded_intersections':report['geometry_samples']>0 and report['padded_intersection_samples']==0,
                'tracking':report['max_path_deviation_m']<=thresholds.get('max_path_deviation_m',.3),
                'stopped':report['stop_verified']}
        if task['yaw_mode']=='fixed':checks['fixed_heading_max']=report['max_fixed_heading_error_rad']<=thresholds.get('fixed_heading_max_rad',.1)
        if task.get('pure_lateral',False):checks['lateral_cross_track']=report['max_cross_track_m']<=thresholds.get('pure_lateral_cross_track_m',.15)
        if task.get('pure_rotation',False):checks['rotation_position_drift']=report['max_rotation_drift_m']<=thresholds.get('rotation_drift_m',.06)
        if task.get('require_combined_motion',False):
            checks['combined_motion_duration']=report['combined_motion_sim_s']>=thresholds.get('combined_motion_min_s',.5)
            checks['tracked_yaw_reference']=report['max_tracked_yaw_error_rad']<=thresholds.get('max_yaw_error_rad',.35)
        missing=required-set(checks)
        if missing:raise ValueError('Unimplemented declared constraints: '+','.join(sorted(missing)))
        report['checks']=checks;report['passed']=all(checks.values());report['status']='completed'
        tracking_ratio=(report['max_cross_track_m']/thresholds.get('pure_lateral_cross_track_m',.15) if task.get('pure_lateral') else
                        report['max_rotation_drift_m']/thresholds.get('rotation_drift_m',.06) if task.get('pure_rotation') else
                        report['max_path_deviation_m']/thresholds.get('max_path_deviation_m',.3))
        heading_ratio=(report['max_fixed_heading_error_rad']/thresholds.get('fixed_heading_max_rad',.1) if task['yaw_mode']=='fixed' else
                       report['final_yaw_error_rad']/thresholds.get('final_yaw_rad',.12))
        components={'completion':report['waypoints_reached']/len(waypoints),
                    'terminal_pose':math.exp(-.5*((report['final_position_error_m']/thresholds.get('final_position_m',.15))**2+(report['final_yaw_error_rad']/thresholds.get('final_yaw_rad',.12))**2)),
                    'heading':math.exp(-heading_ratio**2),'tracking':math.exp(-tracking_ratio**2)}
        report['score_components']=components
        report['score_100']=100*sum(task.get('score_weights',{}).get(k,0.)*v for k,v in components.items()) if report['passed'] else 0.
        exit_code=0 if report['passed'] else 1
    except (KeyboardInterrupt,InterruptedError) as error:
        report['status']='interrupted';report['exception']=str(error)
    except Exception as error:
        report['status']='failed';report['exception']=type(error).__name__+': '+str(error)
        import traceback;report['traceback']=traceback.format_exc()
    finally:
        if node:
            try:
                deadline=time.monotonic()+.5
                while time.monotonic()<deadline:
                    node.send(np.zeros(3));rclpy.spin_once(node,timeout_sec=.02)
                node.destroy_node()
            except Exception:pass
        if rclpy and rclpy.ok():rclpy.shutdown()
        report['finished_unix_s']=time.time();report['exit_code']=exit_code;write(opts.output,report)
        print(json.dumps({k:report.get(k) for k in ['task_id','passed','status','terminal_reason','final_position_error_m','elapsed_sim_s','exception']}),flush=True)
    return exit_code


if __name__=='__main__':sys.exit(main())

#!/usr/bin/env python3
"""Start independent Gazebo episodes for frozen baseline and trained policies."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


DOMAIN='79'
PARTITION='qianli_omni_eval_v1'


def write(path,data):
    path=Path(path);temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def members(pgid):
    result=[]
    for proc in Path('/proc').iterdir():
        if proc.name.isdigit():
            try:
                if os.getpgid(int(proc.name))==pgid and (proc/'stat').read_text().split()[2]!='Z':result.append(proc)
            except (ProcessLookupError,FileNotFoundError,PermissionError):pass
    return result


def verify(group):
    for proc in group:
        env=(proc/'environ').read_bytes().split(b'\0')
        if ('ROS_DOMAIN_ID='+DOMAIN).encode() not in env or ('GZ_PARTITION='+PARTITION).encode() not in env:
            raise RuntimeError('Unsafe process group identity: '+proc.name)


def stop(process):
    if process is None:return
    group=members(process.pid)
    if not group:return
    verify(group);os.killpg(process.pid,signal.SIGINT)
    for escalation,seconds in [(signal.SIGTERM,6.),(signal.SIGKILL,4.)]:
        deadline=time.monotonic()+seconds
        while members(process.pid) and time.monotonic()<deadline:time.sleep(.1)
        group=members(process.pid)
        if not group:break
        verify(group);os.killpg(process.pid,escalation)
    try:process.wait(timeout=2.)
    except subprocess.TimeoutExpired:pass


def main():
    from ament_index_python.packages import get_package_share_directory
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--tasks')
    parser.add_argument('--variant',default='test_101')
    parser.add_argument('--subset',nargs='+')
    parser.add_argument('--policies',nargs='+',default=['baseline','trained'])
    parser.add_argument('--policy-directory',type=Path,help='Optional newly trained policy files; default is installed frozen policies')
    args=parser.parse_args()
    share=Path(get_package_share_directory('qianli_omni_learning'))
    tasks_path=Path(args.tasks) if args.tasks else share/'config/omni_tasks.json'
    config=json.loads(tasks_path.read_text())
    jobs=[t for t in config['evaluation_tasks'] if args.subset is None or t['id'] in args.subset]
    if not jobs:raise ValueError('No matching evaluation tasks')
    if args.variant not in config['evaluation_variants']:raise ValueError('Comparison must use a declared held-out variant')
    manifest=Path(get_package_share_directory('qianli_training_scenarios'))/'generated'/args.variant/'manifest.json'
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    lock=(out/'run.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    domain_lock=Path('/tmp/qianli_omni_eval_v1.lock').open('w')
    fcntl.flock(domain_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=DOMAIN,GZ_PARTITION=PARTITION,DISPLAY=':98',
                                  LIBGL_ALWAYS_SOFTWARE='1',QT_QPA_PLATFORM='xcb',XDG_RUNTIME_DIR='/run/user/1000')
    env.pop('XAUTHORITY',None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    if subprocess.run(['xdpyinfo','-display',':98'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
        raise RuntimeError('Xvfb :98 must be ready before GPU lidar starts')
    run_id=uuid.uuid4().hex
    report={'schema_version':1,'status':'running','started_unix_s':time.time(),'run_id':run_id,
            'variant':args.variant,'tasks_file':str(tasks_path),'episodes':[],
            'domain':DOMAIN,'partition':PARTITION,'policy_uses_ground_truth':False,
            'shared_limits':{'translation_m_s':.25,'yaw_rad_s':.6}}
    path=out/'comparison.json';write(path,report)
    launch=None;evaluator=None
    def interrupt(_signum,_frame):raise InterruptedError('Comparison interrupted')
    signal.signal(signal.SIGTERM,interrupt)
    try:
        for label in args.policies:
            policy=(args.policy_directory if args.policy_directory else share/'policies')/(label+'_policy.json')
            if not policy.is_file():raise FileNotFoundError(policy)
            for task in jobs:
                name=label+'__'+task['id'];episode=out/name;episode.mkdir(exist_ok=True)
                command=['ros2','launch','qianli_bringup','training.launch.py',
                         'variant:='+args.variant,'nav2:=false','slam:=false','gui:=false','rviz:=false',
                         'headless_rendering:=false','spawn_x:='+str(task['start'][0]),
                         'spawn_y:='+str(task['start'][1]),'spawn_yaw:='+str(task['start'][2]),'spawn_z:=0.0']
                print('starting',name,flush=True)
                with (episode/'bringup.log').open('w') as stream:
                    launch=subprocess.Popen(command,env=env,start_new_session=True,stdin=subprocess.DEVNULL,
                                            stdout=stream,stderr=subprocess.STDOUT)
                (out/'active_launch.pid').write_text(str(launch.pid))
                eval_command=['ros2','run','qianli_omni_learning','omni_ros_eval.py','--tasks',str(tasks_path),
                              '--task',task['id'],'--manifest',str(manifest),'--policy',str(policy),
                              '--output',str(episode/'result.json'),'--run-id',run_id]
                evaluation_started=time.time()
                with (episode/'evaluation.log').open('w') as stream:
                    evaluator=subprocess.Popen(eval_command,env=env,start_new_session=True,stdin=subprocess.DEVNULL,
                                               stdout=stream,stderr=subprocess.STDOUT)
                (out/'active_evaluator.pid').write_text(str(evaluator.pid))
                try:evaluator.wait(timeout=90+max(150,task['max_sim_time_s']*4))
                except subprocess.TimeoutExpired:stop(evaluator)
                evaluation_returncode=evaluator.returncode
                evaluator=None
                stop(launch);launch=None
                result_path=episode/'result.json'
                if result_path.exists():
                    result=json.loads(result_path.read_text())
                    if result.get('run_id')!=run_id or result.get('started_unix_s',0)<evaluation_started-1.:
                        raise RuntimeError('Stale result from previous run: '+str(result_path))
                    if evaluation_returncode not in [0,1] or bool(result.get('passed'))!=(evaluation_returncode==0):
                        raise RuntimeError('Evaluator return code/result disagree: '+str(evaluation_returncode))
                    result['policy_label']=label;report['episodes'].append(result)
                else:
                    result={'policy_label':label,'task_id':task['id'],'passed':False,'status':'missing_report'}
                    report['episodes'].append(result)
                write(path,report)
                print('finished',name,'passed',result.get('passed'),'seconds',result.get('elapsed_sim_s'),
                      'reason',result.get('terminal_reason',result.get('exception')),flush=True)
                if result.get('status')!='completed' or result.get('terminal_reason') in ['invalid_lidar','stale_telemetry','wall_timeout']:
                    raise RuntimeError('Sensor/telemetry failure; comparison stopped')
        summary={}
        for label in args.policies:
            rows=[r for r in report['episodes'] if r['policy_label']==label]
            summary[label]={'passed':sum(r['passed'] for r in rows),'requested':len(jobs),
                            'elapsed_sim_s':sum(r.get('elapsed_sim_s',0.) for r in rows),
                            'raw_intersection_samples':sum(r.get('raw_intersection_samples',0) for r in rows),
                            'padded_intersection_samples':sum(r.get('padded_intersection_samples',0) for r in rows),
                            'mean_score_100':sum(r.get('score_100',0) for r in rows)/len(rows)}
        report['summary']=summary;report['status']='completed'
        if 'baseline' in summary and 'trained' in summary:
            denominator=summary['baseline']['elapsed_sim_s']
            report['time_change_percent']=100*(summary['trained']['elapsed_sim_s']/denominator-1) if denominator else None
            report['both_all_passed']=all(s['passed']==s['requested'] for s in summary.values())
    except (KeyboardInterrupt,InterruptedError) as error:
        report['status']='interrupted';report['exception']=str(error)
    except Exception as error:
        report['status']='failed';report['exception']=type(error).__name__+': '+str(error)
        import traceback;report['traceback']=traceback.format_exc()
    finally:
        stop(evaluator);stop(launch)
        report['finished_unix_s']=time.time();write(path,report)
    print('report',path,report['status'],flush=True)
    return 0 if report['status']=='completed' and all(e['passed'] for e in report['episodes']) else 1


if __name__=='__main__':raise SystemExit(main())

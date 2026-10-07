#!/usr/bin/env python3
"""Read-only target checks. Simulation checks never connect to the arm."""

from project_paths import open_video_capture
import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

from project_paths import calibration_path, default_arm_port, default_camera, parts_path, project_path, so101_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', action='store_true', help='Require native Ubuntu 22.04 / Python 3.10')
    parser.add_argument('--ros', action='store_true', help='Verify Humble and installed public modules')
    parser.add_argument('--calibration', action='store_true', help='Require quality-approved intrinsics/extrinsics')
    parser.add_argument('--devices', action='store_true', help='Capture one camera frame and check serial permissions')
    parser.add_argument('--gpu', action='store_true', help='Require NVIDIA driver and actual PyTorch CUDA arithmetic')
    parser.add_argument('--render', action='store_true', help='Create a MuJoCo renderer (set MUJOCO_GL=egl for headless Linux)')
    args=parser.parse_args()
    failures=[]
    report={}
    def check(name, call):
        try:
            report[name]=call()
        except Exception as exc:
            failures.append(name)
            report[name]={'error':str(exc)}

    def target():
        release=Path('/etc/os-release').read_text()
        assert 'ID=ubuntu' in release and 'VERSION_ID="22.04"' in release, 'Ubuntu 22.04 required'
        assert sys.version_info[:2]==(3,10), 'Python 3.10 required'
        return dict(python=platform.python_version(), kernel=platform.release())
    if args.target: check('target',target)

    def resources():
        urdf=Path(so101_path('urdf/so101.urdf'))
        assert urdf.is_file(), f'Missing {urdf}'
        parts=Path(parts_path())
        meshes=list(parts.glob('*.stl'))
        assert len(meshes)==68, f'Expected 68 convex meshes, found {len(meshes)}'
        assert (parts/'manifest.txt').is_file(), 'Missing convex mesh manifest'
        return dict(urdf=str(urdf), convex_meshes=len(meshes))
    check('resources',resources)
    report['dependencies']={}
    for name in ('numpy','scipy','opencv-python','mujoco','ikpy','torch','gymnasium','stable-baselines3'):
        try: report['dependencies'][name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: pass

    def ros():
        assert os.environ.get('ROS_DISTRO')=='humble', 'Source Humble first'
        import rclpy
        import so101_bringup.driver_node as driver
        import so101_bringup.ik_node as ik
        # Check imports in a fresh interpreter, outside this tool's source
        # bootstrap, using the sourced workspace's installed package paths.
        installed=json.loads(subprocess.check_output(
            [sys.executable,project_path('scripts/tools/check_grasp_install.py')],text=True))
        from ament_index_python.packages import get_package_share_directory
        share=Path(get_package_share_directory('so101_bringup'))
        assert (share/'config/driver_params.yaml').is_file()
        assert (share/'launch/ik_demo.launch.py').is_file()
        return dict(driver=driver.__file__,ik=ik.__file__,installed=installed,rclpy=rclpy.__file__)
    if args.ros: check('ros',ros)

    def calibration():
        sys.path.insert(0,project_path('qianli_ws/src/qianli_vision/scripts'))
        from block_pipeline import load_intrinsics,load_extrinsics
        k,d,meta=load_intrinsics(calibration_path('camera_intrinsics.yaml'))
        assert k is not None, meta
        ext,error=load_extrinsics(calibration_path('extrinsic.txt'))
        assert error is None, error
        return dict(intrinsics=meta,extrinsics=ext)
    if args.calibration: check('calibration',calibration)

    def devices():
        import cv2
        port=Path(default_arm_port())
        assert port.exists() and os.access(port,os.R_OK|os.W_OK), f'Serial permissions/path invalid: {port}'
        camera=default_camera()
        capture=open_video_capture(camera)
        try: ok,frame=capture.read()
        finally: capture.release()
        assert ok and frame is not None, f'Camera capture failed: {camera}'
        return dict(port=str(port), camera=camera, frame=list(frame.shape))
    if args.devices: check('devices',devices)

    def gpu():
        import torch
        driver=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],text=True).strip()
        assert torch.cuda.is_available(), 'PyTorch CUDA unavailable'
        matrix=torch.arange(64,dtype=torch.float32,device='cuda').reshape(8,8)
        product=matrix@matrix.T
        torch.cuda.synchronize()
        torch.testing.assert_close(product.cpu(),matrix.cpu()@matrix.cpu().T)
        return dict(driver=driver,torch=torch.__version__,cuda=torch.version.cuda,
                    device=torch.cuda.get_device_name(0),arithmetic=True)
    if args.gpu: check('gpu',gpu)

    def render():
        import mujoco
        sys.path.insert(0,project_path('qianli_ws/src/qianli_vision/scripts'))
        import sim_mesh_gripper as scene
        model=scene.build(0.04)
        data=mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        with mujoco.Renderer(model,height=128,width=128) as renderer:
            renderer.update_scene(data)
            image=renderer.render()
        assert image.shape==(128,128,3)
        return dict(shape=list(image.shape),backend=os.environ.get('MUJOCO_GL','default'))
    if args.render: check('render',render)
    report['failed_checks']=failures
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return bool(failures)


if __name__=='__main__':
    raise SystemExit(main())

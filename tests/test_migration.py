"""Migration boundaries: relocatable paths, real calibration and archived assets."""
import hashlib
import json
import os
from pathlib import Path
import sys
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'qianli_ws/src/qianli_vision'),
               str(ROOT/'qianli_ws/src/qianli_vision/scripts'), str(ROOT/'scripts/tools')]
from qianli_vision import runtime_paths as paths
import migration_assets as assets


def test_paths_follow_relocated_checkout_and_explicit_overrides(tmp_path, monkeypatch):
    monkeypatch.setenv('QI_PROJECT_ROOT', str(tmp_path/'relocated project'))
    monkeypatch.setenv('QI_CALIB_DIR', str(tmp_path/'measured calibration'))
    monkeypatch.setenv('QI_SO101_PKG', str(tmp_path/'explicit model'))
    monkeypatch.setenv('QI_PARTS_DIR', str(tmp_path/'explicit parts'))
    assert Path(paths.project_path('config')) == tmp_path/'relocated project/config'
    assert Path(paths.calibration_path('extrinsic.txt')) == tmp_path/'measured calibration/extrinsic.txt'
    assert Path(paths.so101_path('urdf/so101.urdf')) == tmp_path/'explicit model/urdf/so101.urdf'
    assert Path(paths.parts_path()) == tmp_path/'explicit parts'
    # Explicit missing config must not select a different robot's limits.
    assert Path(paths.driver_params_path()) == tmp_path/'explicit model/config/driver_params.yaml'
    monkeypatch.setenv('QI_DRIVER_CONFIG', str(tmp_path/'live.yaml'))
    assert Path(paths.driver_params_path()) == tmp_path/'live.yaml'
    monkeypatch.setenv('QI_CAMERA', '/dev/v4l/by-id/camera-video-index0')
    assert paths.default_camera() == '/dev/v4l/by-id/camera-video-index0'
    assert paths.camera_source('2') == 2


@pytest.mark.parametrize('invalid', ['missing', 'quality_ok=0', 'quality_ok=nan',
                                     'grid_origin_x=nan', 'grid_theta_deg=inf'])
def test_calibration_rejects_unknown_or_nonfinite_pose(tmp_path, invalid):
    pytest.importorskip('cv2')
    pytest.importorskip('mujoco')
    import block_pipeline as pipeline
    import sim_grasp as scene
    path = tmp_path/'extrinsic.txt'
    lines = dict(grid_origin_x='0.2', grid_origin_y='0.05', grid_origin_z='-0.06909',
                 grid_theta_deg='15', quality_ok='1')
    path.write_text('\n'.join(k+'='+v for k,v in lines.items()), encoding='utf-8')
    assert scene.load_extrinsics(str(path))[1] is None
    assert scene.BOARD_ORIGIN is not None
    if invalid == 'missing':
        path.unlink()
    else:
        k,v = invalid.split('=')
        lines[k]=v
        path.write_text('\n'.join(k+'='+v for k,v in lines.items()), encoding='utf-8')
    assert pipeline.load_extrinsics(str(path))[0] is None
    assert scene.load_extrinsics(str(path))[0] is None
    assert scene.BOARD_ORIGIN is scene.BOARD_YAW is None


@pytest.mark.parametrize('size', [0.02, 0.04])
@pytest.mark.parametrize('has_board', [False, True])
@pytest.mark.parametrize('model', ['historical', 'ros'])
def test_simulation_fixed_seed_with_optional_board(size, has_board, model, monkeypatch):
    pytest.importorskip('mujoco')
    pytest.importorskip('gymnasium')
    import numpy as np
    import sim_grasp as scene
    import rl_env
    monkeypatch.delenv('QI_SO101_PKG', raising=False)
    urdf = (paths.project_path('dual_twin/urdf/so101.urdf') if model == 'historical'
            else paths.robot_urdf_path())
    monkeypatch.setattr(scene, 'URDF', urdf)
    monkeypatch.setattr(scene, 'BOARD_ORIGIN', (0.2, 0.05) if has_board else None)
    monkeypatch.setattr(scene, 'BOARD_YAW', 0.0 if has_board else None)
    env = rl_env.GraspEnv(obj_size=size, seed=73)
    try:
        obs, _ = env.reset(seed=73)
        replay, _ = env.reset(seed=73)
        np.testing.assert_array_equal(obs, replay)
        assert env.model.geom('board').id >= 0 if has_board else 'board' not in [env.model.geom(i).name for i in range(env.model.ngeom)]
        for _ in range(4):
            obs, reward, _, _, _ = env.step(np.zeros(6, dtype=np.float32))
            assert obs.shape == (22,) and np.isfinite(obs).all() and np.isfinite(reward)
    finally:
        env.close()


def write_bundle(path, files):
    manifest = {'format':1, 'missing_calibration':[], 'files':{
        k:{'size':len(v), 'sha256':hashlib.sha256(v).hexdigest()} for k,v in files.items()}}
    with zipfile.ZipFile(path,'w') as archive:
        for k,v in files.items(): archive.writestr(k,v)
        archive.writestr('manifest.json',json.dumps(manifest))


def test_bundle_roundtrip_and_refuses_overwrite(tmp_path):
    bundle=tmp_path/'assets.zip'
    write_bundle(bundle, {'calibration/extrinsic.txt':b'measured values', 'driver-source/node.py':b'original source'})
    destination=tmp_path/'raw'
    calib=tmp_path/'calib'
    assets.restore_assets(bundle,destination,calib)
    assets.restore_assets(bundle,destination,calib)
    assert (calib/'extrinsic.txt').read_bytes() == b'measured values'
    (calib/'extrinsic.txt').write_bytes(b'new measurement')
    with pytest.raises(ValueError,match='Existing file differs'):
        assets.restore_assets(bundle,destination,calib)
    assert (calib/'extrinsic.txt').read_bytes() == b'new measurement'


def test_bundle_detects_corruption_and_unsafe_paths(tmp_path):
    bundle=tmp_path/'assets.zip'
    write_bundle(bundle,{'../escaped.txt':b'bad'})
    with pytest.raises(ValueError,match='Unsafe'): assets.verify_assets(bundle)
    write_bundle(bundle,{'calibration/extrinsic.txt':b'original'})
    with zipfile.ZipFile(bundle) as archive:
        manifest=json.loads(archive.read('manifest.json'))
    with zipfile.ZipFile(bundle,'w') as archive:
        archive.writestr('calibration/extrinsic.txt',b'tampered')
        archive.writestr('manifest.json',json.dumps(manifest))
    with pytest.raises(ValueError,match='Checksum'): assets.verify_assets(bundle)


@pytest.mark.skipif(os.name == 'nt', reason='Linux sysfs interface names contain colons')
def test_usb_discovery_tracks_port_and_refuses_another_chip(tmp_path):
    import arm_usb_watchdog as watchdog
    sysroot=tmp_path/'sys'
    device=sysroot/'devices/pci/usb3/3-4'
    interface=device/'3-4:1.2'
    tty=interface/'tty/ttyACM7'
    tty.mkdir(parents=True)
    driver=sysroot/'bus/usb/drivers/cdc_acm'
    driver.mkdir(parents=True)
    link=sysroot/'class/tty/ttyACM7/device'
    link.parent.mkdir(parents=True)
    try:
        link.symlink_to(tty,target_is_directory=True)
        (interface/'driver').symlink_to(driver,target_is_directory=True)
    except OSError:
        pytest.skip('Directory symlinks unavailable; discovery is also tested on Linux')
    for name,value in {'idVendor':'1a86','idProduct':'55d3','busnum':'3','devnum':'11'}.items():
        (device/name).write_text(value)
    (interface/'bInterfaceNumber').write_text('02')
    result=watchdog.discover_usb_device('/dev/ttyACM7',sysroot,tmp_path/'dev')
    assert result['node']==tmp_path/'dev/bus/usb/003/011'
    assert result['interface']=='3-4:1.2'
    assert result['driver']==driver
    (device/'idProduct').write_text('9999')
    assert watchdog.discover_usb_device('/dev/ttyACM7',sysroot,tmp_path/'dev') is None

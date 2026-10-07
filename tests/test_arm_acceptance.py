"""Read-only acceptance must not mistake successful communication for readiness."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts/tools'), str(ROOT/'qianli_ws/src/qianli_vision'),
               str(ROOT/'qianli_ws/src/qianli_vision/scripts')]


@pytest.mark.parametrize('case', ['ready', 'outside', 'missing', 'energized'])
def test_readonly_report_rejects_unready_arm_without_register_writes(tmp_path, monkeypatch, case):
    pytest.importorskip('yaml')
    import arm_readonly_check as check
    from project_paths import driver_params_path
    import yaml

    params = yaml.safe_load(Path(driver_params_path()).read_text(encoding='utf-8'))['so101_driver']['ros__parameters']
    positions = [(lo+hi)//2 for lo, hi in zip(params['raw_min'], params['raw_max'])]
    if case == 'outside':
        positions[3] = params['raw_min'][3] - 1
    calls = []

    class ReadOnlyBus:
        IDS = (1, 2, 3, 4, 5, 6)

        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def read_word(self, servo_id, address):
            calls.append((servo_id, address))
            assert address == 56
            if case == 'missing' and servo_id == 3:
                raise TimeoutError('servo did not respond')
            return positions[servo_id-1]

        def read_byte(self, servo_id, address):
            calls.append((servo_id, address))
            assert address == 40
            return int(case == 'energized' and servo_id == 1)

    # This transport exposes no write operations; an accidental write fails the test.
    monkeypatch.setitem(sys.modules, 'so101_bringup.servo_protocol',
                        SimpleNamespace(FeetechSerialBus=ReadOnlyBus))
    monkeypatch.setitem(sys.modules, 'arm_usb_watchdog',
                        SimpleNamespace(discover_usb_device=lambda port: None))
    destination = tmp_path/'nested'/'report.json'
    monkeypatch.setattr(sys, 'argv', ['check', '--port', '/dev/test-arm', '--output', str(destination)])
    assert check.main() == (0 if case == 'ready' else 1)
    report = json.loads(destination.read_text())
    assert report['communication_ok'] == (case != 'missing')
    assert report['within_joint_limits'] == (case not in ('outside', 'missing'))
    assert report['torque_disabled'] == (case not in ('energized', 'missing'))
    assert report['passed'] == (case == 'ready')
    assert len(report['servos']) == 6
    assert all(address in (40, 56) for _, address in calls)


@pytest.mark.parametrize('bad_measurement', [False, True])
def test_composed_calibration_has_measured_height_and_respects_quality(tmp_path, monkeypatch, bad_measurement):
    pytest.importorskip('cv2')
    import numpy as np
    import compose_extrinsic as compose
    from qianli_vision.calibration import load_extrinsics

    # Synthetic test geometry stays in tmp_path and is never installed on a robot.
    board_cam = tmp_path/'camera.npz'
    np.savez(board_cam, rvec=np.zeros(3), tvec=np.array([0., 0., 0.5]),
             K=np.array([[485., 0., 320.], [0., 485., 240.], [0., 0., 1.]]),
             D=np.zeros(5), n_frames=10, tvec_std=np.zeros(3), rows=5, cols=7, cell=.033)
    origin = np.array([.2, .05, compose.TABLE_Z_MEASURED + .0005])
    grids = np.array([[0., 0.], [9.9, 0.], [0., 6.6], [9.9, 6.6], [3.3, 3.3]])
    contacts = np.column_stack((grids/100., np.zeros(len(grids)))) + origin
    if bad_measurement:
        contacts[0, 0] += .03
    marks = tmp_path/'marks.json'
    marks.write_text(json.dumps([dict(grid_cm=g.tolist(), contact_m=p.tolist())
                                 for g, p in zip(grids, contacts)]))
    calib_dir = tmp_path/'persistent calibration'
    monkeypatch.setenv('QI_CALIB_DIR', str(calib_dir))
    output = calib_dir/'camera'/'extrinsic.yaml'
    monkeypatch.setattr(sys, 'argv', ['compose', '--board-cam', str(board_cam),
                                    '--marks', str(marks), '--out', str(output)])
    assert compose.main() == (2 if bad_measurement else 0)
    for path in (output, calib_dir/'extrinsic.txt'):
        values, reason = load_extrinsics(path)
        if bad_measurement:
            assert values is None and 'quality_ok=0' in reason
        else:
            assert reason is None
            assert values['grid_origin_z'] == pytest.approx(origin[2], abs=1e-9)

"""Physical calibration must use current direct feedback and real collector progress."""
from pathlib import Path
import json
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'qianli_ws/src/qianli_vision'),
               str(ROOT/'qianli_ws/src/qianli_vision/scripts')]


@pytest.mark.parametrize('invalid', ['none', 'sim', 'motion_allowed', 'enabled', 'holding',
                                    'fault', 'stale_status', 'stale_joints', 'missing_joint', 'nan_joint'])
def test_physical_touch_rejects_unverified_or_stale_feedback(invalid):
    from qianli_vision.calibration import CALIBRATION_JOINTS, physical_collection_error
    status = dict(mode='direct', allow_motion=False, enabled=False, holding=False, fault='')
    joints = dict.fromkeys(CALIBRATION_JOINTS, 0.1)
    ages = [0.1, 0.1]
    assert physical_collection_error(status, joints, *ages) is None
    if invalid == 'none':
        status = None
    elif invalid == 'sim':
        status['mode'] = 'sim'
    elif invalid == 'motion_allowed':
        status['allow_motion'] = True
    elif invalid in ('enabled', 'holding'):
        status[invalid] = True
    elif invalid == 'fault':
        status['fault'] = 'serial disconnected'
    elif invalid == 'stale_status':
        ages[0] = 5.0
    elif invalid == 'stale_joints':
        ages[1] = 5.0
    elif invalid == 'missing_joint':
        joints.pop('gripper')
    elif invalid == 'nan_joint':
        joints['elbow_flex'] = float('nan')
    assert physical_collection_error(status, joints, *ages) is not None


def test_frontend_tracks_current_collector_output_and_clears_old_rejection():
    pytest.importorskip('cv2')
    from touch_frontend import parse_progress, mark_request_error
    text = '▶ 第 1/5 点：grid (0.0, 0.0) cm\n   ⛔ 固定爪顶端高度不合格\n'
    state = parse_progress(text)
    assert state['cur'] == 1 and state['done'] == 0 and '高度不合格' in state['note']
    assert mark_request_error(state, False) is not None
    assert mark_request_error(state, True) is None
    text += '   ✅ 固定爪顶端 = (0.2, 0.05, -0.069)\n'
    state = parse_progress(text)
    assert state['done'] == 1 and state['note'] == ''
    assert mark_request_error(state, True) is not None
    text += '▶ 第 2/5 点：grid (9.9, 0.0) cm\n'
    state = parse_progress(text)
    assert state['cur'] == 2 and state['done'] == 1
    assert mark_request_error(state, True) is None
    assert mark_request_error(parse_progress(''), True) is not None


def test_frontend_preserves_legacy_contact_log_support():
    pytest.importorskip('cv2')
    from touch_frontend import parse_progress
    state = parse_progress('▶ 第 1/1 点：grid (0, 0)\n✅ 接触点 = (0.2, 0.05, -0.069)')
    assert state['cur'] == state['total'] == state['done'] == 1


def test_frontend_keeps_pending_click_and_publishes_complete_point_number(tmp_path):
    pytest.importorskip('cv2')
    from touch_frontend import create_mark_request
    trigger = tmp_path/'request.json'
    create_mark_request(trigger, 1)
    with pytest.raises(FileExistsError):
        create_mark_request(trigger, 2)
    assert json.loads(trigger.read_text()) == {'point': 1}
    assert list(tmp_path.iterdir()) == [trigger]

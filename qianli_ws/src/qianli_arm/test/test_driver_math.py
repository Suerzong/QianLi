import numpy as np
from types import SimpleNamespace
from unittest.mock import Mock, patch

from so101_bringup.driver_node import So101DriverNode, NOMINAL_LIMITS


def math_node():
    """Initialize the state used by pure math tests without constructing ROS."""
    node = object.__new__(So101DriverNode)
    node.joint_lower = np.array([bounds[0] for bounds in NOMINAL_LIMITS])
    node.joint_upper = np.array([bounds[1] for bounds in NOMINAL_LIMITS])
    node._report_clip = Mock()
    return node


def test_tracking_compensation_is_zero_at_target():
    node = math_node()
    node.q_output = np.array([0.2, -0.3, 0.1, 0.0, 0.4, 0.2])
    node.q_current = node.q_output.copy()
    node.tracking_gain = 1.0
    node.max_tracking_compensation = 0.05
    assert np.allclose(node._compensated_output(), node.q_output)


def test_tracking_compensation_leads_and_is_bounded():
    node = math_node()
    node.q_output = np.array([0.4, -0.4, 0.2, -0.2, 0.3, 0.1])
    node.q_current = np.zeros(6)
    node.tracking_gain = 1.0
    node.max_tracking_compensation = 0.05
    expected = node.q_output + np.array(
        [0.05, -0.05, 0.05, -0.05, 0.05, 0.05])
    assert np.allclose(node._compensated_output(), expected)


def test_reconnect_runs_in_background_and_deduplicates_attempts():
    node = math_node()
    node.mode = 'direct'
    node.bus = None
    node.reconnect_in_progress = False
    node._reconnect_worker = Mock()

    with patch('so101_bringup.driver_node.threading.Thread') as thread:
        node._reconnect_if_needed()
        node._reconnect_if_needed()

    assert node.reconnect_in_progress is True
    thread.assert_called_once()
    thread.return_value.start.assert_called_once()


def test_reconnect_is_not_started_with_live_bus():
    node = math_node()
    node.mode = 'direct'
    node.bus = object()
    node.reconnect_in_progress = False

    with patch('so101_bringup.driver_node.threading.Thread') as thread:
        node._reconnect_if_needed()

    thread.assert_not_called()


def test_encoder_feedback_is_not_clipped_at_soft_limit():
    node = math_node()
    node.zero_raw = [2078, 1980, 3076, 2035, 3053, 2030]
    node.direction = [1] * 6

    measured = node._raw_to_joint([2078, 3197, 3076, 2035, 3053, 2030])

    assert measured[1] > 1.74533


def test_recovery_raw_conversion_can_hold_pose_outside_soft_limit():
    node = math_node()
    node.zero_raw = [2078, 1980, 3076, 2035, 3053, 2030]
    node.direction = [1] * 6
    node.raw_min = [826, 842, 1974, 954, 1264, 1916]
    node.raw_max = [3330, 3118, 4095, 3116, 4095, 3168]
    raw = [2078, 3197, 3076, 2035, 3053, 2030]
    measured = node._raw_to_joint(raw)

    assert node._joint_to_raw(
        measured, enforce_soft_limits=False) == raw
    assert node._joint_to_raw(measured)[1] == node.raw_max[1]


def test_recovery_compensation_never_moves_farther_outward():
    node = math_node()
    node.q_output = np.array([0.0, 1.82, 0.0, 0.0, 0.0, 0.0])
    node.q_current = np.array([0.0, 1.70, 0.0, 0.0, 0.0, 0.0])
    node.tracking_gain = 1.0
    node.max_tracking_compensation = 0.08
    node.recovering_outside_limits = True

    compensated = node._compensated_output()

    assert compensated[1] == node.q_output[1]


def test_integral_compensation_accumulates_and_is_bounded():
    node = math_node()
    node.q_output = np.full(6, 0.1)
    node.q_current = np.zeros(6)
    node.tracking_gain = 0.0
    node.tracking_integral_gain = 2.0
    node.tracking_integral = np.zeros(6)
    node.control_rate = 10.0
    node.max_tracking_compensation = 0.05
    node.recovering_outside_limits = False

    first = node._compensated_output()
    for _ in range(20):
        final = node._compensated_output()

    assert np.allclose(first, 0.12)
    assert np.allclose(final, 0.15)
    assert np.allclose(node.tracking_integral, 0.05)


def test_new_joint_target_clears_stale_integral():
    node = math_node()
    node.q_target = np.zeros(6)
    node.tracking_integral = np.full(6, 0.03)
    node.last_command_time = 0.0
    msg = SimpleNamespace(
        name=['shoulder_lift'],
        position=[-0.02],
    )

    node._on_command(msg)

    assert node.q_target[1] == -0.02
    assert np.allclose(node.tracking_integral, 0.0)

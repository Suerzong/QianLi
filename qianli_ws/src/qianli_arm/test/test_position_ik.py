from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ikpy.chain import Chain

from so101_bringup.ik_node import (
    DEFAULT_HOME_POSITIONS,
    IK_JOINT_NAMES,
    JOINT_NAMES,
    So101IkNode,
)


def position_only_chain():
    urdf = Path(__file__).parents[4] / 'dual_twin' / 'urdf' / 'so101.urdf'
    node = object.__new__(So101IkNode)
    node.orientation_mode = 'none'
    prepared = So101IkNode._prepare_ik_urdf(node, urdf)
    node.chain = Chain.from_urdf_file(
        prepared, base_elements=['base_link'], name='test_so101')
    So101IkNode._apply_active_mask(node)
    return node.chain


def test_position_only_ik_keeps_wrist_roll_fixed():
    chain = position_only_chain()
    link_names = [getattr(link, 'name', None) for link in chain.links]
    wrist_index = link_names.index('wrist_roll')

    # Representative bent pose captured during the first hardware test.
    start = np.zeros(len(chain.links))
    angles = {
        'shoulder_pan': -0.001534,
        'shoulder_lift': -0.4387,
        'elbow_flex': -0.1212,
        'wrist_flex': 0.2608,
        'wrist_roll': 1.4267,
    }
    for name, value in angles.items():
        start[link_names.index(name)] = value

    start_position = chain.forward_kinematics(start)[:3, 3]
    for axis in range(3):
        for displacement in (-0.003, 0.003):
            target = start_position.copy()
            target[axis] += displacement
            solution = chain.inverse_kinematics(
                target_position=target,
                initial_position=start,
                orientation_mode=None,
            )
            achieved = chain.forward_kinematics(solution)[:3, 3]
            assert solution[wrist_index] == start[wrist_index]
            assert np.linalg.norm(achieved - target) < 1e-5
            assert np.max(np.abs(solution - start)) < 0.05


def test_position_only_mask_has_four_pose_joints():
    chain = position_only_chain()
    active = {
        chain.links[index].name
        for index, enabled in enumerate(chain.active_links_mask)
        if enabled
    }
    assert active == set(IK_JOINT_NAMES) - {'wrist_roll'}


def test_single_solve_round_trip_does_not_drift_posture():
    chain = position_only_chain()
    link_names = [getattr(link, 'name', None) for link in chain.links]
    start = np.zeros(len(chain.links))
    angles = {
        'shoulder_pan': -0.0046,
        'shoulder_lift': 0.7470,
        'elbow_flex': 0.2224,
        'wrist_flex': 0.1917,
        'wrist_roll': 1.4389,
    }
    for name, value in angles.items():
        start[link_names.index(name)] = value

    origin = chain.forward_kinematics(start)[:3, 3]
    displaced = origin + np.array([0.0, 0.005, 0.0])
    outbound = chain.inverse_kinematics(
        target_position=displaced,
        initial_position=start,
        orientation_mode=None,
    )
    returned = chain.inverse_kinematics(
        target_position=origin,
        initial_position=outbound,
        orientation_mode=None,
    )

    assert np.max(np.abs(returned - start)) < 0.001
    assert np.linalg.norm(
        chain.forward_kinematics(returned)[:3, 3] - origin) < 1e-5


def test_enabled_feedback_does_not_erase_command_trajectory():
    node = object.__new__(So101IkNode)
    link_names = ['Base link', *IK_JOINT_NAMES, 'tool_tip_joint']
    node.chain = SimpleNamespace(
        links=[SimpleNamespace(name=name) for name in link_names])
    node.q_current = np.array([0.0, 0.4, 0.3, 0.2, 0.1, 0.0, 0.0])
    outbound_before_feedback = node.q_current.copy()
    node.q_measured = np.zeros(7)
    node.joint_state_synced = True
    node.have_driver_status = True
    node.motion_enabled = True
    msg = SimpleNamespace(
        name=list(JOINT_NAMES),
        position=[-0.1, -0.2, -0.3, -0.4, -0.5, 0.0],
    )

    node._on_joint_state(msg)

    assert np.array_equal(node.q_current, outbound_before_feedback)
    assert np.allclose(node.q_measured[1:6], msg.position[:5])


def test_default_home_is_geometry_orthogonal():
    chain = position_only_chain()
    link_names = [getattr(link, 'name', None) for link in chain.links]
    q = np.zeros(len(chain.links))
    for name, value in zip(JOINT_NAMES, DEFAULT_HOME_POSITIONS):
        if name in link_names:
            q[link_names.index(name)] = value
    transforms = chain.forward_kinematics(q, full_kinematics=True)
    upper = transforms[3][:3, 3] - transforms[2][:3, 3]
    forearm = transforms[4][:3, 3] - transforms[3][:3, 3]
    upper /= np.linalg.norm(upper)
    forearm /= np.linalg.norm(forearm)

    assert upper[2] > 0.999999
    assert forearm[0] > 0.999999
    assert abs(float(np.dot(upper, forearm))) < 1e-6

#!/usr/bin/env python3
"""Safe command/state bridge for the SO-ARM101 follower arm.

Modes:
  sim    - rate-limited command echo for RViz testing; no hardware access.
  direct - USB controller connected directly to the HX-30HM servo bus.

Direct mode starts torque-disabled.  Motion can only be enabled when both
``allow_motion`` and ``calibrated`` are true, followed by an explicit call to
``/arm/enable``.  A stale command or repeated read error disables torque.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker

from .servo_protocol import FeetechSerialBus, radians_to_raw, raw_to_radians


JOINT_NAMES = (
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
)

# Joint-space limits are NOT hardcoded here any more.  They are derived from
# ``raw_min``/``raw_max`` (the measured mechanical envelope in servo counts)
# together with ``zero_raw``/``direction``.  The old module-level constants
# duplicated the URDF numbers, so the driver and the model could disagree
# silently; deriving them means a joint's soft limit has exactly one source.
#
# The values below are only used to sanity-check a ``sim``-mode launch that has
# no hardware attached.
NOMINAL_LIMITS = (
    (-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69),
    (-1.65806, 1.65806), (-2.74385, 2.84121), (-0.174533, 1.74533),
)


class So101DriverNode(Node):

    def __init__(self):
        super().__init__('so101_driver')

        self.declare_parameter('mode', 'sim')
        self.declare_parameter('port', os.environ.get('QI_ARM_PORT', '/dev/ttyACM0'))
        self.declare_parameter('baud', 1_000_000)
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('calibrated', False)
        self.declare_parameter('preserve_torque', False)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('state_rate', 10.0)
        self.declare_parameter('command_write_rate', 15.0)
        self.declare_parameter('max_joint_speed', 0.35)
        self.declare_parameter('tracking_compensation_gain', 0.0)
        self.declare_parameter('tracking_integral_gain', 0.0)
        self.declare_parameter('max_tracking_compensation', 0.05)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('reject_out_of_range', False)
        self.declare_parameter('zero_raw', [2048] * 6)
        self.declare_parameter('direction', [1] * 6)
        self.declare_parameter(
            'raw_min', [674, 856, 787, 838, 505, 2034])
        self.declare_parameter(
            'raw_max', [3260, 3266, 3069, 3199, 3987, 3233])

        self.mode = str(self.get_parameter('mode').value).lower()
        if self.mode not in ('sim', 'direct'):
            raise ValueError("mode must be 'sim' or 'direct'")

        self.allow_motion = bool(self.get_parameter('allow_motion').value)
        self.calibrated = bool(self.get_parameter('calibrated').value)
        self.preserve_torque = bool(
            self.get_parameter('preserve_torque').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.state_rate = float(self.get_parameter('state_rate').value)
        self.command_write_rate = float(
            self.get_parameter('command_write_rate').value)
        self.max_speed = float(self.get_parameter('max_joint_speed').value)
        self.tracking_gain = float(
            self.get_parameter('tracking_compensation_gain').value)
        self.tracking_integral_gain = float(
            self.get_parameter('tracking_integral_gain').value)
        self.max_tracking_compensation = float(
            self.get_parameter('max_tracking_compensation').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        if min(self.control_rate, self.state_rate, self.command_write_rate,
               self.max_speed,
               self.command_timeout) <= 0.0:
            raise ValueError('rates, speed and timeout must be positive')
        if (self.tracking_gain < 0.0 or self.tracking_integral_gain < 0.0
                or self.max_tracking_compensation < 0.0):
            raise ValueError('tracking compensation values must be non-negative')

        self.add_on_set_parameters_callback(self._on_tuning_parameters)

        self.zero_raw = self._int_array('zero_raw')
        self.direction = self._int_array('direction')
        self.raw_min = self._int_array('raw_min')
        self.raw_max = self._int_array('raw_max')
        if any(d not in (-1, 1) for d in self.direction):
            raise ValueError('each direction must be -1 or 1')
        if any(lo >= hi for lo, hi in zip(self.raw_min, self.raw_max)):
            raise ValueError('every raw_min must be less than raw_max')
        # A goal is written as one 12-bit word.  A window that pokes outside
        # this range used to be silently truncated, which is how wrist_roll
        # lost 71 degrees of real travel.  Refuse to start instead.
        if any(lo < 0 or hi > 4095
               for lo, hi in zip(self.raw_min, self.raw_max)):
            raise ValueError(
                'raw_min/raw_max must lie within the 12-bit goal range '
                '0..4095; a wider window needs the servo re-zeroed so the '
                f'travel fits one turn (got {self.raw_min}..{self.raw_max})')
        self._derive_joint_limits()
        self.reject_out_of_range = bool(
            self.get_parameter('reject_out_of_range').value)
        self.clip_events = 0
        self.clip_last = None
        self.clip_seen = []
        self._last_clip_log = {}

        self.enabled = self.mode == 'sim'
        self.holding = False
        self.bus = None
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.fault = ''
        self.read_failures = 0
        self.last_command_time = time.monotonic()
        self.last_write_time = 0.0
        self.last_written_raw = None
        self.reconnect_in_progress = False
        self.recovering_outside_limits = False
        self.q_current = np.zeros(6, dtype=float)
        self.q_output = self.q_current.copy()
        self.q_target = self.q_current.copy()
        self.tracking_integral = np.zeros(6, dtype=float)
        self.last_raw = None

        if self.mode == 'direct':
            try:
                self._connect_direct_bus()
            except Exception as exc:  # noqa: BLE001 - hardware can be off
                self.bus = None
                self.enabled = False
                self.holding = False
                self.fault = f'initial servo connection pending: {exc}'
                self.get_logger().warning(
                    f'{self.fault}; background reconnect will continue')

        self.pub_state = self.create_publisher(
            JointState, '/joint_states', 10)
        self.pub_status = self.create_publisher(
            String, '/arm/status', 10)
        self.pub_marker = self.create_publisher(
            Marker, '/arm/status_marker', 1)
        self.sub_command = self.create_subscription(
            JointState, '/joint_commands', self._on_command, 10)
        self.srv_enable = self.create_service(
            SetBool, '/arm/enable', self._on_enable)
        self.srv_stop = self.create_service(
            Trigger, '/arm/stop', self._on_stop)

        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_tick)
        if self.mode == 'direct':
            self.state_timer = self.create_timer(
                1.0 / self.state_rate, self._read_direct_state)
            self.reconnect_timer = self.create_timer(
                1.0, self._reconnect_if_needed)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'SO-101 driver ready: mode={self.mode}, enabled={self.enabled}, '
            f'allow_motion={self.allow_motion}, calibrated={self.calibrated}')
        if self.mode == 'direct' and not self.allow_motion:
            self.get_logger().warning(
                'Direct mode is READ-ONLY. Relaunch with allow_motion:=true '
                'after calibration to permit explicit enabling.')

    def _on_tuning_parameters(self, parameters):
        """Validate and apply the small set of safe runtime tuning knobs."""
        max_speed = self.max_speed
        tracking_gain = self.tracking_gain
        integral_gain = self.tracking_integral_gain
        max_compensation = self.max_tracking_compensation

        try:
            for parameter in parameters:
                if parameter.name == 'max_joint_speed':
                    max_speed = float(parameter.value)
                elif parameter.name == 'tracking_compensation_gain':
                    tracking_gain = float(parameter.value)
                elif parameter.name == 'tracking_integral_gain':
                    integral_gain = float(parameter.value)
                elif parameter.name == 'max_tracking_compensation':
                    max_compensation = float(parameter.value)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(
                successful=False, reason=f'invalid tuning value: {exc}')

        if max_speed <= 0.0:
            return SetParametersResult(
                successful=False, reason='max_joint_speed must be positive')
        if tracking_gain < 0.0:
            return SetParametersResult(
                successful=False,
                reason='tracking_compensation_gain must be non-negative')
        if integral_gain < 0.0:
            return SetParametersResult(
                successful=False,
                reason='tracking_integral_gain must be non-negative')
        if max_compensation < 0.0:
            return SetParametersResult(
                successful=False,
                reason='max_tracking_compensation must be non-negative')

        changed = (
            max_speed != self.max_speed
            or tracking_gain != self.tracking_gain
            or integral_gain != self.tracking_integral_gain
            or max_compensation != self.max_tracking_compensation
        )
        reset_integral = integral_gain != self.tracking_integral_gain
        self.max_speed = max_speed
        self.tracking_gain = tracking_gain
        self.tracking_integral_gain = integral_gain
        self.max_tracking_compensation = max_compensation
        if reset_integral and hasattr(self, 'tracking_integral'):
            self.tracking_integral.fill(0.0)
        if changed:
            self.get_logger().info(
                'Runtime tuning updated: '
                f'max_joint_speed={self.max_speed:.3f} rad/s, '
                f'tracking_gain={self.tracking_gain:.3f}, '
                f'integral_gain={self.tracking_integral_gain:.3f}/s, '
                'max_tracking_compensation='
                f'{self.max_tracking_compensation:.3f} rad')
        return SetParametersResult(successful=True)

    def _int_array(self, name):
        values = [int(v) for v in self.get_parameter(name).value]
        if len(values) != 6:
            raise ValueError(f'{name} must contain six values')
        return values

    def _derive_joint_limits(self):
        """Derive joint-space limits from the raw servo envelope.

        ``raw_min``/``raw_max`` describe what the mechanism can physically do,
        so they are the authority.  Deriving the joint limits here (instead of
        repeating the URDF numbers as module constants) removes the class of
        bug where the driver and the model drift apart without anyone noticing.
        """
        lower, upper = [], []
        for zero, direction, lo, hi in zip(
                self.zero_raw, self.direction, self.raw_min, self.raw_max):
            low = raw_to_radians(lo, zero, direction)
            high = raw_to_radians(hi, zero, direction)
            lower.append(min(low, high))
            upper.append(max(low, high))
        self.joint_lower = np.asarray(lower, dtype=float)
        self.joint_upper = np.asarray(upper, dtype=float)
        self.raw_lower = np.asarray(
            [min(lo, hi) for lo, hi in zip(self.raw_min, self.raw_max)],
            dtype=float)
        self.raw_upper = np.asarray(
            [max(lo, hi) for lo, hi in zip(self.raw_min, self.raw_max)],
            dtype=float)

        rows = []
        for name, zero, lo, hi, low, high in zip(
                JOINT_NAMES, self.zero_raw, self.raw_min, self.raw_max,
                lower, upper):
            rows.append(
                f'  {name:<14} raw [{lo:5d}, {hi:5d}]'
                f'  = [{math.degrees(low):+7.2f}, {math.degrees(high):+7.2f}] deg'
                f'   zero_raw={zero}')
        self.get_logger().info(
            'Joint envelope (raw counts are the single source of truth):\n'
            + '\n'.join(rows))

        # The IK node is what enforces the URDF/model limits; the driver must
        # not be *wider* than the model or it would happily drive the arm into
        # a pose the model considers invalid.
        for name, low, high, nominal in zip(
                JOINT_NAMES, lower, upper, NOMINAL_LIMITS):
            over_lo = nominal[0] - low
            over_hi = high - nominal[1]
            if over_lo > math.radians(1.0) or over_hi > math.radians(1.0):
                self.get_logger().warn(
                    f'{name}: driver envelope is wider than the URDF nominal '
                    f'by {math.degrees(max(over_lo, 0.0)):+.1f} deg / '
                    f'{math.degrees(max(over_hi, 0.0)):+.1f} deg; '
                    'the IK node will still clip to the model limits')

    def _report_clip(self, kind, index, requested, applied, limit_name, limit):
        """Record and announce a swallowed target instead of hiding it.

        Silently clipping a command is how a wrong soft limit turns into
        "the arm just stops there for no reason".  Every clip is counted,
        logged (rate limited) and published on /arm/status.
        """
        name = JOINT_NAMES[index]
        now = time.monotonic()
        self.clip_events += 1
        self.clip_last = {
            'kind': kind, 'joint': name,
            'requested_rad': round(float(requested), 5),
            'applied_rad': round(float(applied), 5),
            'limit': limit_name,
            'swallowed_deg': round(math.degrees(float(requested - applied)), 2),
        }
        tag = f'{kind}:{name}'
        if tag not in self.clip_seen:
            self.clip_seen.append(tag)
            del self.clip_seen[:-8]
        key = (kind, index)
        if now - self._last_clip_log.get(key, -1e9) > 2.0:
            self._last_clip_log[key] = now
            self.get_logger().warning(
                f'{kind}: {name} requested {math.degrees(requested):+.2f} deg '
                f'but {limit_name} allows {math.degrees(limit):+.2f} deg -> '
                f'applied {math.degrees(applied):+.2f} deg '
                f'({math.degrees(requested - applied):+.2f} deg swallowed, '
                f'total {self.clip_events} clip events)')

    def _connect_direct_bus(self):
        # Keep the candidate transport private until the complete handshake
        # succeeds.  State/control timers must never observe a half-connected
        # bus while a background reconnect is in progress.
        bus = FeetechSerialBus(
            self.port, self.baud, timeout_s=0.08)
        try:
            if self.preserve_torque:
                # Calibration can leave the arm locked at its measured zero
                # pose.  Adopt that hold without a release/re-energize cycle,
                # but never send motion until /arm/enable is called.
                torque_states = bus.read_torque_states()
                if any(value != 1 for value in torque_states):
                    raise RuntimeError(
                        'preserve_torque requires all six servos already '
                        f'locked: {torque_states}')
                holding = True
            else:
                # Normal direct-mode startup always forces a known safe state.
                bus.set_torque(False)
                torque_states = bus.read_torque_states()
                if any(torque_states):
                    raise RuntimeError(
                        f'failed to verify torque disabled: {torque_states}')
                holding = False
            raw = bus.read_positions()
        except Exception:
            bus.close()
            raise

        self.bus = bus
        self.holding = holding
        self.last_raw = list(raw)
        self.q_current = self._raw_to_joint(raw)
        self.q_output = self.q_current.copy()
        self.q_target = self.q_current.copy()
        state = 'existing torque hold preserved' if self.holding else 'torque disabled'
        self.get_logger().info(
            f'Connected to six servos on {self.port}; {state}')

    def _disconnect_for_reconnect(self, reason):
        self.enabled = False
        self.holding = False
        self.fault = reason
        bus, self.bus = self.bus, None
        if bus is not None:
            try:
                bus.set_torque(False)
            except Exception:  # noqa: BLE001 - device may already be gone
                pass
            try:
                bus.close()
            except Exception:  # noqa: BLE001
                pass
        self.get_logger().error(reason)

    def _reconnect_if_needed(self):
        if (self.mode != 'direct' or self.bus is not None
                or self.reconnect_in_progress):
            return
        self.reconnect_in_progress = True
        threading.Thread(
            target=self._reconnect_worker,
            name='so101-servo-reconnect',
            daemon=True,
        ).start()

    def _reconnect_worker(self):
        """Reconnect outside the ROS executor so status/stop stay responsive."""
        try:
            # A reconnected controller always comes back motion-disabled.
            preserve_torque = self.preserve_torque
            self.preserve_torque = False
            try:
                self._connect_direct_bus()
            finally:
                self.preserve_torque = preserve_torque
            self.read_failures = 0
            self.fault = ''
            self.get_logger().warning(
                'Servo bus reconnected; motion remains disabled. '
                'Use /arm/enable after checking the workspace.')
        except Exception as exc:  # noqa: BLE001
            if self.bus is not None:
                try:
                    self.bus.close()
                except Exception:  # noqa: BLE001
                    pass
            self.bus = None
            self.get_logger().warning(f'servo reconnect pending: {exc}')
        finally:
            self.reconnect_in_progress = False

    def _on_command(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warning(
                'Rejected joint command: name/position lengths differ')
            return
        candidate = self.q_target.copy()
        seen = set()
        for name, value in zip(msg.name, msg.position):
            if name not in JOINT_NAMES or name in seen or not math.isfinite(value):
                self.get_logger().warning(
                    f'Rejected invalid joint command entry: {name!r}={value!r}')
                return
            seen.add(name)
            candidate[JOINT_NAMES.index(name)] = float(value)

        lower = getattr(self, 'joint_lower', None)
        upper = getattr(self, 'joint_upper', None)
        if lower is None:
            lower = np.asarray([n[0] for n in NOMINAL_LIMITS], dtype=float)
            upper = np.asarray([n[1] for n in NOMINAL_LIMITS], dtype=float)
        next_target = np.clip(candidate, lower, upper)

        # Anything the envelope changed is reported, never hidden.
        clipped = np.nonzero(np.abs(next_target - candidate) > 1e-6)[0]
        if len(clipped):
            if self.reject_out_of_range:
                for i in clipped:
                    self._report_clip('rejected', int(i), candidate[i],
                                      next_target[i], 'soft limit',
                                      next_target[i])
                self.get_logger().warning(
                    'reject_out_of_range is enabled: the whole command was '
                    f'dropped because {len(clipped)} joint(s) exceeded the '
                    'configured envelope')
                return
            for i in clipped:
                i = int(i)
                limit = lower[i] if candidate[i] < next_target[i] else upper[i]
                self._report_clip('clamped', i, candidate[i], next_target[i],
                                  'soft limit', limit)

        if (np.max(np.abs(next_target - self.q_target)) > 1e-5
                and hasattr(self, 'tracking_integral')):
            # Integral learned for the previous static load must not oppose a
            # new small Cartesian step.  Rebuild it after the new target has
            # settled instead of carrying stale compensation across targets.
            self.tracking_integral.fill(0.0)
        self.q_target = next_target
        self.last_command_time = time.monotonic()

    def _on_enable(self, request, response):
        if not request.data:
            self._disable('disabled by operator')
            response.success = True
            response.message = 'motion disabled'
            return response
        if self.mode == 'sim':
            self.enabled = True
            response.success = True
            response.message = 'simulation enabled'
            return response
        if self.bus is None:
            response.success = False
            response.message = 'servo bus is disconnected; waiting for reconnect'
            return response
        if not self.allow_motion:
            response.success = False
            response.message = 'allow_motion is false; direct driver is read-only'
            return response
        if not self.calibrated:
            response.success = False
            response.message = 'calibrated is false; zero/direction must be verified first'
            return response
        if self.fault:
            response.success = False
            response.message = f'clear the fault by restarting: {self.fault}'
            return response
        try:
            # Read once synchronously and hold those exact encoder counts.  A
            # powerless arm can sag slightly beyond a URDF soft limit; using
            # a clipped joint angle here would create a jump at torque-on.
            hold_raw = self.bus.read_positions()
            # 舵机停在量程端点时会读回 4096/4097/4104…（或多圈边界的负值）。
            # 这是**合法的物理读数**（Present_Position 是 16 位、可多圈），
            # 不该因此拒绝使能 —— 否则某个关节一旦停在机械死点上，
            # 整条臂就永远使能不了（实测踩到：elbow 读回 4104）。
            #
            # 但 Goal_Position 只有一个 12 位字，所以**写回时必须夹到 0..4095**。
            # 注意夹紧而不是取模：4104 → 4095 只差 9 个计数，而 4104 % 4096 = 8
            # 会让舵机反向转将近一整圈，非常危险。
            if any(raw < -4096 or raw > 8191 for raw in hold_raw):
                raise RuntimeError(
                    f'encoder value far out of range while enabling: {hold_raw}')
            hold_goal = [min(4095, max(0, raw)) for raw in hold_raw]
            if hold_goal != hold_raw:
                self.get_logger().warning(
                    'encoder read outside 0..4095 while enabling; goal clamped '
                    f'(not wrapped): {hold_raw} -> {hold_goal}')
            self.last_raw = list(hold_raw)
            self.q_current = self._raw_to_joint(hold_raw)
            self.q_output = self.q_current.copy()
            self.q_target = self.q_current.copy()
            self.tracking_integral.fill(0.0)
            self.recovering_outside_limits = bool(np.any(
                (self.q_output < self.joint_lower)
                | (self.q_output > self.joint_upper)))
            self.bus.write_positions(hold_goal)
            self.last_written_raw = list(hold_goal)
            self.last_write_time = time.monotonic()
            self.bus.set_torque(True)
            torque_states = self.bus.read_torque_states()
            if any(value != 1 for value in torque_states):
                raise RuntimeError(
                    f'failed to verify torque enabled: {torque_states}')
            self.enabled = True
            self.holding = True
            self.last_command_time = time.monotonic()
        except Exception as exc:  # noqa: BLE001 - hardware safety boundary
            self._fault(f'enable failed: {exc}')
            response.success = False
            response.message = self.fault
            return response
        response.success = True
        response.message = 'motion enabled; command watchdog is active'
        return response

    def _on_stop(self, _request, response):
        self._disable('emergency stop requested')
        response.success = True
        response.message = 'torque disabled'
        return response

    def _control_tick(self):
        max_step = self.max_speed / self.control_rate
        delta = np.clip(self.q_target - self.q_output, -max_step, max_step)
        self.q_output += delta

        if self.mode == 'sim':
            self.q_current = self.q_output.copy()
            self._publish_joint_state()
            return
        if not self.enabled:
            return
        if time.monotonic() - self.last_command_time > self.command_timeout:
            self._disable('command watchdog timeout')
            return
        try:
            if self.recovering_outside_limits and np.all(
                    (self.q_output >= self.joint_lower)
                    & (self.q_output <= self.joint_upper)):
                self.recovering_outside_limits = False
                self.get_logger().info(
                    'All joints recovered inside configured soft limits')
            raw_target = self._joint_to_raw(
                self._compensated_output(),
                enforce_soft_limits=not self.recovering_outside_limits,
            )
            now = time.monotonic()
            changed = (
                self.last_written_raw is None or
                any(abs(current - previous) > 1
                    for current, previous in zip(
                        raw_target, self.last_written_raw))
            )
            if changed and now - self.last_write_time < (
                    1.0 / self.command_write_rate):
                return
            # Smart servos hold the last goal internally.  Avoid flooding the
            # CH343 adapter with an identical packet every control tick; send
            # at full rate only while the raw goal is changing, plus 2 Hz
            # keepalive writes while stationary.
            if not changed and now - self.last_write_time < 0.5:
                return
            self.bus.write_positions(raw_target)
            self.last_written_raw = list(raw_target)
            self.last_write_time = now
        except Exception as exc:  # noqa: BLE001 - hardware safety boundary
            self._disconnect_for_reconnect(
                f'position write failed; reconnect required: {exc}')

    def _compensated_output(self):
        error = self.q_output - self.q_current
        integral_gain = getattr(self, 'tracking_integral_gain', 0.0)
        if not hasattr(self, 'tracking_integral'):
            self.tracking_integral = np.zeros_like(error)
        if integral_gain > 0.0:
            self.tracking_integral += (
                integral_gain * error / self.control_rate)
            np.clip(
                self.tracking_integral,
                -self.max_tracking_compensation,
                self.max_tracking_compensation,
                out=self.tracking_integral,
            )
        else:
            self.tracking_integral.fill(0.0)
        correction = np.clip(
            self.tracking_gain * error + self.tracking_integral,
            -self.max_tracking_compensation,
            self.max_tracking_compensation,
        )
        candidate = self.q_output + correction
        if getattr(self, 'recovering_outside_limits', False):
            # Permit a rate-limited path from the measured out-of-range pose
            # back into the normal envelope, but never command farther out.
            recovery_lower = np.minimum(self.joint_lower, self.q_output)
            recovery_upper = np.maximum(self.joint_upper, self.q_output)
            return np.clip(candidate, recovery_lower, recovery_upper)
        return np.clip(candidate, self.joint_lower, self.joint_upper)

    def _read_direct_state(self):
        if self.bus is None:
            return
        try:
            raw = self.bus.read_positions()
            self.last_raw = list(raw)
            self.q_current = self._raw_to_joint(raw)
            self.read_failures = 0
            self._publish_joint_state()
        except Exception as exc:  # noqa: BLE001 - hardware safety boundary
            self.read_failures += 1
            self.get_logger().warning(
                f'servo state read failed ({self.read_failures}/3): {exc}')
            if self.read_failures >= 3:
                self._disconnect_for_reconnect(
                    f'three consecutive state read failures; '
                    f'reconnect required: {exc}')

    def _joint_to_raw(self, joints, enforce_soft_limits=True):
        minimums = self.raw_min if enforce_soft_limits else [0] * 6
        maximums = self.raw_max if enforce_soft_limits else [4095] * 6
        result = []
        for index, (q, zero, direction, minimum, maximum) in enumerate(zip(
                joints, self.zero_raw, self.direction, minimums, maximums)):
            wanted = round(zero + direction * q * 4096.0 / math.tau)
            raw = max(minimum, min(maximum, wanted))
            if raw != wanted and enforce_soft_limits:
                # The joint-space clip should already have kept this inside the
                # window, so reaching here means the two limits disagree.
                self._report_clip(
                    'raw clamp', index,
                    raw_to_radians(wanted, zero, direction),
                    raw_to_radians(raw, zero, direction),
                    'raw window',
                    raw_to_radians(
                        minimum if wanted < raw else maximum, zero, direction))
            result.append(raw)
        return result

    def _raw_to_joint(self, raw_positions):
        result = [
            raw_to_radians(raw, zero, direction)
            for raw, zero, direction in zip(
                raw_positions, self.zero_raw, self.direction)
        ]
        # Encoder feedback is factual state, not a command.  Preserve values
        # beyond a soft limit so enable can hold the exact powerless pose and
        # then recover inward without a torque-on jump.
        return np.asarray(result, dtype=float)

    def _disable(self, reason):
        self.enabled = False
        self.holding = False
        self.recovering_outside_limits = False
        if hasattr(self, 'tracking_integral'):
            self.tracking_integral.fill(0.0)
        if self.bus is not None:
            try:
                self.bus.set_torque(False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'failed to disable torque: {exc}')
        self.get_logger().warning(reason)

    def _fault(self, reason):
        self.fault = reason
        self._disable(reason)

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = self.q_current.tolist()
        self.pub_state.publish(msg)

    def _publish_status(self):
        age = time.monotonic() - self.last_command_time
        payload = {
            'mode': self.mode,
            'enabled': self.enabled,
            'allow_motion': self.allow_motion,
            'calibrated': self.calibrated,
            'holding': self.holding,
            'recovering_to_limits': self.recovering_outside_limits,
            'max_tracking_error_rad': round(float(np.max(
                np.abs(self.q_output - self.q_current))), 4),
            'max_integral_compensation_rad': round(float(np.max(
                np.abs(self.tracking_integral))), 4),
            'command_age_s': round(age, 3),
            'fault': self.fault,
            # Targets the configured envelope changed rather than executed.
            # A non-zero counter here means something asked for a pose the
            # limits refuse; it is never silently absorbed.
            'clip_events': self.clip_events,
            'clip_seen': list(self.clip_seen),
            'clip_last': self.clip_last,
            'reject_out_of_range': self.reject_out_of_range,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_status.publish(msg)

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'arm_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.42
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.035
        marker.color.a = 1.0
        if self.fault:
            marker.text = f'FAULT: {self.fault}'
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.1
        elif self.mode == 'sim':
            marker.text = 'SIMULATION'
            marker.color.r, marker.color.g, marker.color.b = 0.2, 0.8, 1.0
        elif self.enabled:
            marker.text = 'HARDWARE ENABLED'
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.45, 0.0
        elif self.holding:
            marker.text = 'HARDWARE HOLDING - MOTION DISABLED'
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.7, 0.0
        else:
            marker.text = 'HARDWARE DISABLED'
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.9, 0.1
        self.pub_marker.publish(marker)

    def destroy_node(self):
        if self.bus is not None:
            self.enabled = False
            self.holding = False
            try:
                self.bus.set_torque(False)
            except (Exception, KeyboardInterrupt):  # noqa: BLE001
                pass
            try:
                self.bus.close()
            except (Exception, KeyboardInterrupt):  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = So101DriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()

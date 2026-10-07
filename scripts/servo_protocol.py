"""Small, dependency-light Feetech STS protocol helpers.

HX-30HM servos used by the follower arm are compatible with the Feetech
STS packet format.  WRITE packets do not return a status packet on the
tested arm, so all writes in this module are deliberately tx-only.
"""

from __future__ import annotations

import math
import time
from typing import Iterable, Sequence


HEADER = b'\xff\xff'
BROADCAST_ID = 0xFE
INST_READ = 0x02
INST_SYNC_WRITE = 0x83

REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56
# ---- STS3215 内存表里的反馈寄存器（原来只定义了三个，闭环控制要用这些）----
# 地址来自 Feetech STS 系列内存表：0x38 位置之后依次是速度、负载、电压、温度。
# 注意 Present_Load 的编码：低 10 位是幅值 0~1000（=0~100.0%），
# **bit10 (0x400) 是方向位**，直接当无符号数读会把反向负载读成 1000+。
REG_PRESENT_LOAD = 60          # 0x3C
REG_PRESENT_VOLTAGE = 62       # 0x3E  单位 0.1V
REG_PRESENT_TEMPERATURE = 63   # 0x3F  单位 ℃
REG_MOVING = 65                # 0x41  0=已到位 1=正在动
REG_PRESENT_CURRENT = 69       # 0x45  部分固件才有，读不到要能降级

RAW_PER_REVOLUTION = 4096.0


def decode_load(raw: int) -> tuple[float, bool]:
    """把 Present_Load 原始字解成 (百分比 0~100, 方向是否为正)。

    实测坑：这个寄存器**不是**普通无符号数。bit10 是方向位，
    幅值在低 10 位。当无符号读时反向负载会变成 1024+ 的值，
    看起来像"过载 102%"而其实方向和大小都不对。
    """
    mag = raw & 0x3FF
    positive = not (raw & 0x400)
    return mag / 10.0, positive


class ServoProtocolError(RuntimeError):
    """Raised when a servo packet is malformed or reports an error."""


def packet_checksum(core: Iterable[int]) -> int:
    """Return the STS ones-complement checksum for ID..parameters."""
    return (~sum(core)) & 0xFF


def instruction_packet(servo_id: int, instruction: int,
                       parameters: Iterable[int] = ()) -> bytes:
    params = bytes(parameters)
    if not 0 <= servo_id <= 0xFE:
        raise ValueError('servo_id must be in 0..254')
    length = len(params) + 2  # instruction + checksum
    if length > 0xFF:
        raise ValueError('packet is too long')
    core = bytes((servo_id, length, instruction)) + params
    return HEADER + core + bytes((packet_checksum(core),))


def read_packet(servo_id: int, address: int, size: int) -> bytes:
    if not 1 <= size <= 0xFF:
        raise ValueError('read size must be in 1..255')
    return instruction_packet(servo_id, INST_READ, (address, size))


def sync_write_packet(address: int, item_size: int,
                      ids: Sequence[int], data: bytes) -> bytes:
    if not ids:
        raise ValueError('at least one servo id is required')
    if item_size <= 0 or len(data) != len(ids) * item_size:
        raise ValueError('data length does not match ids and item_size')
    params = bytearray((address, item_size))
    for index, servo_id in enumerate(ids):
        if not 1 <= servo_id <= 0xFD:
            raise ValueError('sync-write servo ids must be in 1..253')
        start = index * item_size
        params.append(servo_id)
        params.extend(data[start:start + item_size])
    return instruction_packet(BROADCAST_ID, INST_SYNC_WRITE, params)


def parse_status_packet(packet: bytes, expected_id: int,
                        expected_data_size: int) -> bytes:
    expected_length = expected_data_size + 2  # error + data + checksum
    if len(packet) != expected_length + 4:
        raise ServoProtocolError(
            f'bad status length: got {len(packet)}, expected {expected_length + 4}')
    if packet[:2] != HEADER:
        raise ServoProtocolError('bad status header')
    if packet[2] != expected_id:
        raise ServoProtocolError(
            f'wrong servo id: got {packet[2]}, expected {expected_id}')
    if packet[3] != expected_length:
        raise ServoProtocolError(
            f'wrong payload length: got {packet[3]}, expected {expected_length}')
    if packet_checksum(packet[2:-1]) != packet[-1]:
        raise ServoProtocolError('bad status checksum')
    # HX-30HM/STS-compatible servos set bit 4 while torque is enabled.
    # It is a state flag, not a fault.  Treat the remaining documented bits
    # (voltage, encoder, temperature, current and overload) as errors.
    error = packet[4] & ~0x10
    if error:
        raise ServoProtocolError(
            f'servo {expected_id} reported error flags 0x{error:02x}')
    return packet[5:-1]


def radians_to_raw(angle: float, zero_raw: int, direction: int,
                   minimum: int, maximum: int) -> int:
    if direction not in (-1, 1):
        raise ValueError('direction must be -1 or 1')
    raw = round(zero_raw + direction * angle * RAW_PER_REVOLUTION / math.tau)
    return max(minimum, min(maximum, raw))


def raw_to_radians(raw: int, zero_raw: int, direction: int) -> float:
    if direction not in (-1, 1):
        raise ValueError('direction must be -1 or 1')
    return direction * (raw - zero_raw) * math.tau / RAW_PER_REVOLUTION


class FeetechSerialBus:
    """Minimal serial transport used by the ROS direct-driver mode."""

    IDS = (1, 2, 3, 4, 5, 6)

    def __init__(self, port: str, baud: int = 1_000_000,
                 timeout_s: float = 0.02):
        import serial

        self._timeout_s = timeout_s
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout_s,
            write_timeout=timeout_s,
            exclusive=True,
        )

    @property
    def is_open(self) -> bool:
        return bool(self._serial.is_open)

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def set_torque(self, enabled: bool) -> None:
        values = bytes((1 if enabled else 0 for _ in self.IDS))
        self._serial.write(sync_write_packet(
            REG_TORQUE_ENABLE, 1, self.IDS, values))
        self._serial.flush()

    def write_positions(self, raw_positions: Sequence[int]) -> None:
        if len(raw_positions) != len(self.IDS):
            raise ValueError('six raw positions are required')
        data = bytearray()
        for raw in raw_positions:
            if not 0 <= raw <= 4095:
                raise ValueError(f'raw position out of range: {raw}')
            data.extend((raw & 0xFF, (raw >> 8) & 0xFF))
        self._serial.write(sync_write_packet(
            REG_GOAL_POSITION, 2, self.IDS, bytes(data)))
        self._serial.flush()

    def sync_write_register(self, address: int, item_size: int,
                            servo_ids: Sequence[int], data: bytes) -> None:
        """Write an equally-sized register value to selected servos."""
        self._serial.write(sync_write_packet(
            address, item_size, servo_ids, data))
        self._serial.flush()

    def read_positions(self) -> list[int]:
        return [self.read_word(servo_id, REG_PRESENT_POSITION)
                for servo_id in self.IDS]

    def read_torque_states(self) -> list[int]:
        return [self.read_byte(servo_id, REG_TORQUE_ENABLE)
                for servo_id in self.IDS]

    def read_loads(self) -> list[tuple[float, bool]]:
        """六个舵机的负载 (百分比, 方向)，供"按力合爪"用。"""
        return [decode_load(self.read_word(s, REG_PRESENT_LOAD))
                for s in self.IDS]

    def read_gripper_load(self) -> tuple[float, bool]:
        """只读夹爪（最后一个 ID）的负载 —— 合爪闭环每周期都要读，
        逐个读全部舵机会把周期拖长。"""
        return decode_load(self.read_word(self.IDS[-1], REG_PRESENT_LOAD))

    def read_moving(self) -> list[int]:
        return [self.read_byte(s, REG_MOVING) for s in self.IDS]

    def read_byte(self, servo_id: int, address: int) -> int:
        return self.read_bytes(servo_id, address, 1)[0]

    def read_word(self, servo_id: int, address: int) -> int:
        data = self.read_bytes(servo_id, address, 2)
        return data[0] | (data[1] << 8)

    def read_bytes(self, servo_id: int, address: int, size: int) -> bytes:
        # The tested controller can leave stale bytes after writes.  Clearing
        # them before a READ mirrors the proven stage-1 scripts.
        self._serial.reset_input_buffer()
        request = read_packet(servo_id, address, size)
        self._serial.write(request)
        self._serial.flush()
        reply = self._read_status_packet()
        # Some CH343 controller revisions briefly echo the instruction frame
        # before switching the half-duplex bus to receive.  It is a complete,
        # checksum-valid frame but not the servo status packet.
        if reply == request:
            reply = self._read_status_packet()
        return parse_status_packet(reply, servo_id, size)

    def _read_status_packet(self) -> bytes:
        deadline = time.monotonic() + self._timeout_s
        previous_ff = False
        while time.monotonic() < deadline:
            chunk = self._serial.read(1)
            if not chunk:
                continue
            byte = chunk[0]
            if previous_ff and byte == 0xFF:
                break
            previous_ff = byte == 0xFF
        else:
            raise TimeoutError('servo status header timeout')

        prefix = self._serial.read(2)  # id, length
        if len(prefix) != 2:
            raise TimeoutError('servo status prefix timeout')
        length = prefix[1]
        body = self._serial.read(length)
        if len(body) != length:
            raise TimeoutError('servo status body timeout')
        return HEADER + prefix + body

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.close()

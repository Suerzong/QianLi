import math

import pytest

from so101_bringup.servo_protocol import (
    FeetechSerialBus,
    ServoProtocolError,
    instruction_packet,
    packet_checksum,
    parse_status_packet,
    radians_to_raw,
    raw_to_radians,
    read_packet,
    sync_write_packet,
)


class FakeSerial:
    def __init__(self, incoming):
        self.incoming = bytearray(incoming)
        self.written = bytearray()

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        data = self.incoming[:size]
        del self.incoming[:size]
        return bytes(data)


def test_read_packet_matches_feetech_format():
    packet = read_packet(1, 56, 2)
    assert packet[:7] == bytes((0xFF, 0xFF, 1, 4, 2, 56, 2))
    assert packet[-1] == packet_checksum(packet[2:-1])


def test_sync_write_two_positions():
    packet = sync_write_packet(
        42, 2, (1, 2), bytes((0x00, 0x08, 0x01, 0x08)))
    assert packet[:7] == bytes((0xFF, 0xFF, 0xFE, 10, 0x83, 42, 2))
    assert packet[7:-1] == bytes((1, 0, 8, 2, 1, 8))
    assert packet[-1] == packet_checksum(packet[2:-1])


def test_bus_sync_write_register_uses_selected_ids():
    bus = object.__new__(FeetechSerialBus)
    bus._serial = FakeSerial(b'')

    bus.sync_write_register(21, 1, (2, 3, 4), bytes((32, 32, 32)))

    assert bytes(bus._serial.written) == sync_write_packet(
        21, 1, (2, 3, 4), bytes((32, 32, 32)))


def test_status_packet_validation():
    core = bytes((3, 4, 0, 0x34, 0x12))
    packet = b'\xff\xff' + core + bytes((packet_checksum(core),))
    assert parse_status_packet(packet, 3, 2) == b'\x34\x12'

    corrupted = packet[:-1] + bytes((packet[-1] ^ 1,))
    with pytest.raises(ServoProtocolError, match='checksum'):
        parse_status_packet(corrupted, 3, 2)


def test_status_packet_accepts_torque_enabled_flag():
    core = bytes((3, 4, 0x10, 0x34, 0x12))
    packet = b'\xff\xff' + core + bytes((packet_checksum(core),))
    assert parse_status_packet(packet, 3, 2) == b'\x34\x12'


def test_status_packet_rejects_real_fault_with_torque_enabled():
    core = bytes((3, 4, 0x14, 0x34, 0x12))
    packet = b'\xff\xff' + core + bytes((packet_checksum(core),))
    with pytest.raises(ServoProtocolError, match='0x04'):
        parse_status_packet(packet, 3, 2)


def test_read_skips_ch343_instruction_echo():
    request = read_packet(3, 56, 2)
    core = bytes((3, 4, 0, 0x34, 0x12))
    status = b'\xff\xff' + core + bytes((packet_checksum(core),))
    bus = object.__new__(FeetechSerialBus)
    bus._timeout_s = 0.02
    bus._serial = FakeSerial(request + status)
    assert bus.read_bytes(3, 56, 2) == b'\x34\x12'
    assert bus._serial.written == request


def test_angle_conversion_and_clamp():
    raw = radians_to_raw(math.pi / 2, 2048, 1, 0, 4095)
    assert raw == 3072
    assert raw_to_radians(raw, 2048, 1) == pytest.approx(math.pi / 2)
    assert radians_to_raw(10.0, 2048, 1, 674, 3260) == 3260
    assert radians_to_raw(math.pi / 2, 2048, -1, 0, 4095) == 1024


def test_instruction_packet_rejects_invalid_id():
    with pytest.raises(ValueError):
        instruction_packet(255, 2)

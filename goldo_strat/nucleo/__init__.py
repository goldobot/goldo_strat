from .hal_config import HALConfig
import pb2
import struct


def compute_crc(buffer):
    crc = 0
    for b in buffer:
        x = crc >> 8 ^ b
        x ^= x >> 4
        crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xffff
    return crc


def align_buffer(buff):
    k = len(buff) % 8
    if k == 0:
        return buff
    else:
        return buff + b'\0' * (8 - k)


class BufferBuilder(object):
    def __init__(self):
        self.buff = b''
        self.sections = []

    def push_section(self, section_id, section_buffer):
        offset = len(self.buff)
        self.buff = align_buffer(self.buff + section_buffer)
        self.sections.append((section_id, offset))

    def compile(self):
        header_size = len(self.sections) * 4 + 2
        # compute number of padding bytes to align header on a 8 bytes boundary
        header_padding = header_size % 8
        if header_padding > 0:
            header_padding = 8 - header_padding
        base_offset = header_size + header_padding
        header = struct.pack('H', len(self.sections)) + b''.join(
            [struct.pack('HH', s[0], s[1] + base_offset) for s in self.sections])
        header = header + b'\0' * header_padding
        binary = header + self.buff
        return binary, compute_crc(binary)


class ConfigSection:
    Hal = 0
    RobotGeometry = 1
    Sensors = 2
    RobotSimulator = 3
    Odometry = 4
    PropulsionController = 5
    Servos = 6
    PropulsionTask = 7
    TasksEnable = 8
    Lifts = 9


task_ids = {
    'propulsion': 0,
    'odrive_comm': 1,
    'servos': 2,
    'dynamixels_comm': 3,
    'fpga': 4
}


def _make_tasks_enable_flags(proto):
    flags = 0
    for task in proto.nucleo.enabled_tasks:
        flags |= 1 << task_ids[task]
    return flags


def _compile_sensors(proto):
    buff = struct.pack('<B', len(proto))
    for s in proto:
        buff = buff + struct.pack('<BB', s.type, s.id)
    return buff


def _compile_servos(proto):
    buff = struct.pack('<H', len(proto))
    for s in proto:
        buff = buff + pb2.serialize(s)
    return buff


def _compile_lifts(proto):
    buff = struct.pack('<I', len(proto))
    for s in proto:
        buff = buff + pb2.serialize(s)
    return buff


def compile_config(proto):
    builder = BufferBuilder()
    hal_config = HALConfig(proto.nucleo.hal)
    builder.push_section(ConfigSection.Hal, hal_config.compile())
    builder.push_section(ConfigSection.Sensors, _compile_sensors(proto.nucleo.sensors))
    builder.push_section(ConfigSection.RobotGeometry, pb2.serialize(proto.robot_geometry))
    builder.push_section(ConfigSection.Odometry, pb2.serialize(proto.nucleo.odometry))
    builder.push_section(ConfigSection.PropulsionController, pb2.serialize(proto.nucleo.propulsion))
    builder.push_section(ConfigSection.PropulsionTask, pb2.serialize(proto.nucleo.propulsion_task))
    builder.push_section(ConfigSection.RobotSimulator, pb2.serialize(proto.nucleo.robot_simulator))
    builder.push_section(ConfigSection.Servos, _compile_servos(proto.nucleo.servos))
    builder.push_section(ConfigSection.Lifts, _compile_lifts(proto.nucleo.lifts))
    builder.push_section(ConfigSection.TasksEnable, struct.pack('I', _make_tasks_enable_flags(proto)))

    return builder.compile()

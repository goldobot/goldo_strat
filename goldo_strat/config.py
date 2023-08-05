# Full config format:
# Offsets table
# hal config offset
# robot_config offset
# odometry_config offset
# propulsion_config offset
# arms_config offset
# arms position config offset
# servos config offset
# sequences_config offset
# arms torque offset

def align_buffer(buff):
    k = len(buff) % 8
    if k == 0:
        return buff
    else:
        return buff + b'\0' * (8 - k)


def compile_nucleo_config(config):
    return buff, crc

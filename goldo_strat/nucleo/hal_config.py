import struct
from google.protobuf.json_format import MessageToDict

gpio_port_nums = {
    'PA': 0,
    'PB': 1,
    'PC': 2,
    'PD': 3,
    'PE': 4,
    'PF': 5
}

periph_names = [
    'GPIO',
    'TIM1',
    'TIM2',
    'TIM3',
    'TIM4',
    'TIM6',
    'TIM7',
    'TIM8',
    'TIM15',
    'TIM16',
    'TIM17',
    'TIM20',
    'CAN',
    'I2C1',
    'I2C2',
    'I2C3',
    'SPI1',
    'SPI2',
    'SPI3',
    'USART1',
    'USART2',
    'USART3',
    'UART4',
    'UART5'
]

periph_num = {}
i = 1

for pn in periph_names:
    periph_num[pn] = i
    i += 1


class DeviceType:
    Gpio = 1
    Timer = 2
    Pwm = 3
    Encoder = 4
    Uart = 5
    I2c = 6
    Spi = 7


def align_buffer(buff):
    k = len(buff) % 8
    if k == 0:
        return buff
    else:
        return buff + b'\0' * (8 - k)


class PinConfig:
    def __init__(self, pin_name=None):
        if pin_name not in [None, '']:
            self.port = gpio_port_nums[pin_name[0:2]]
            self.pin = int(pin_name[2:])
        else:
            self.port = 0xff
            self.pin = 0xff

    def compile(self):
        return struct.pack('<BB', self.port, self.pin)


class DeviceConfig:
    def __init__(self, config_dict):
        self.device_id = periph_num[config_dict.get('device', 'GPIO')]


class GpioConfig(DeviceConfig):
    def __init__(self, config_dict=None):
        self.id = config_dict['id']
        self.pin = PinConfig(config_dict['pin'])
        self.dir = {'IN': 0, 'OUT_PP': 1, 'OUT_OD': 2}[config_dict['mode']]
        if config_dict.get('pull', None) == 'UP':
            self.dir |= 0x04

    def compile(self):
        buff = struct.pack('<BB', DeviceType.Gpio, periph_num['GPIO'])
        buff += struct.pack('<BB', self.id, self.dir) + self.pin.compile()
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class TimerConfig(DeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.prescaler = config_dict.get('prescaler', 1)
        self.period = config_dict['period']

    def compile(self):
        buff = struct.pack('<BB', DeviceType.Timer, self.device_id)
        buff += struct.pack('<II', self.prescaler, self.period)
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class PwmConfig(DeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.id = config_dict['id']
        self.channel = {'CH1': 1, 'CH2': 2, 'CH3': 3, 'CH4': 4}[config_dict['channel']]
        self.pin = PinConfig(config_dict['pin'])
        self.n_pin = PinConfig(config_dict.get('n_pin'))
        self.dir_pin = PinConfig(config_dict.get('dir_pin'))

    def compile(self):
        buff = struct.pack('<BB', DeviceType.Pwm, self.device_id)
        buff += struct.pack('<BB', self.id,
                            self.channel) + self.pin.compile() + self.n_pin.compile() + self.dir_pin.compile()
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class EncoderConfig(DeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.id = config_dict['id']
        self.reverse_dir = config_dict.get('reverse_dir', False)
        self.hall_mode = config_dict.get('hall_mode', False)
        self.period = config_dict['period']
        self.ch1_pin = PinConfig(config_dict.get('ch1_pin'))
        self.ch2_pin = PinConfig(config_dict.get('ch2_pin'))
        self.ch3_pin = PinConfig(config_dict.get('ch3_pin'))

    def compile(self):
        flags = 0
        if self.reverse_dir:
            flags |= 0x01
        if self.hall_mode:
            flags |= 0x02
        buff = struct.pack('<BB', DeviceType.Encoder, self.device_id)
        buff += struct.pack('<BB', self.id, flags)
        buff += self.ch1_pin.compile()
        buff += self.ch2_pin.compile()
        buff += self.ch3_pin.compile()
        buff += struct.pack('<H', self.period)
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class IODeviceConfig(DeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.id = config_dict['id']
        self.rx_buffer_size = config_dict.get('rx_buffer_size', 0)
        self.tx_buffer_size = config_dict.get('tx_buffer_size', 0)
        self.rx_blocking = config_dict.get('rx_blocking', False)
        self.tx_blocking = config_dict.get('tx_blocking', False)
        self.rx_dma = config_dict.get('rx_dma', False)
        self.tx_dma = config_dict.get('tx_dma', False)

    @property
    def io_flags(self):
        flags = 0
        if self.rx_blocking:
            flags |= 0x01
        if self.tx_blocking:
            flags |= 0x02
        if self.rx_dma:
            flags |= 0x04
        if self.tx_dma:
            flags |= 0x08
        return flags


class UsartConfig(IODeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.baudrate = config_dict['baudrate']
        self.rx_pin = PinConfig(config_dict['rx_pin'])
        self.tx_pin = PinConfig(config_dict['tx_pin'])
        self.txen_pin = PinConfig(config_dict.get('txen_pin'))

    def compile(self):
        buff = struct.pack('<BB', DeviceType.Uart, self.device_id)
        buff += struct.pack('<BBHHH', self.id, 0, self.rx_buffer_size, self.tx_buffer_size, self.io_flags)
        buff += struct.pack('<I', self.baudrate)
        buff += self.rx_pin.compile() + self.tx_pin.compile() + self.txen_pin.compile()
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class I2cConfig(IODeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.timing = config_dict['timing']
        self.scl_pin = PinConfig(config_dict['scl_pin'])
        self.sda_pin = PinConfig(config_dict['sda_pin'])

    def compile(self):
        buff = struct.pack('<BB', DeviceType.I2c, self.device_id)
        buff += struct.pack('<BBHHH', self.id, 0, self.rx_buffer_size, self.tx_buffer_size, self.io_flags)
        buff += self.scl_pin.compile() + self.sda_pin.compile()
        buff += struct.pack('<I', self.timing)
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class SpiConfig(IODeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.baudrate_prescaler = config_dict['baudrate_prescaler']
        self.sck_pin = PinConfig(config_dict.get('sck_pin'))
        self.mosi_pin = PinConfig(config_dict.get('mosi_pin'))
        self.miso_pin = PinConfig(config_dict.get('miso_pin'))
        self.nss_pin = PinConfig(config_dict.get('nss_pin'))

    def compile(self):
        buff = struct.pack('<BB', DeviceType.Spi, self.device_id)
        buff += struct.pack('<BBHHH', self.id, 0, self.rx_buffer_size, self.tx_buffer_size, self.io_flags)
        buff += self.sck_pin.compile()
        buff += self.mosi_pin.compile()
        buff += self.miso_pin.compile()
        buff += self.nss_pin.compile()
        buff += struct.pack('<H', self.baudrate_prescaler)
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class CanConfig(DeviceConfig):
    def __init__(self, config_dict):
        super().__init__(config_dict)
        self.prescaler = config_dict['prescaler']
        self.period = config_dict['period']

    def compile(self):
        buff = struct.pack('<BB', 7, self.device_id)
        buff += struct.pack('<HH', self.prescaler, self.period)
        buff = struct.pack('<H', len(buff) + 2) + buff
        return buff


class HALConfig:
    def __init__(self, config_proto):
        config_dict = MessageToDict(config_proto, including_default_value_fields=True, preserving_proto_field_name=True)
        self.gpio = [GpioConfig(v) for v in config_dict.get('gpio', [])]
        self.timer = [TimerConfig(v) for v in config_dict.get('timer', [])]
        self.pwm = [PwmConfig(v) for v in config_dict.get('pwm', [])]
        self.encoder = [EncoderConfig(v) for v in config_dict.get('encoder', [])]
        self.usart = [UsartConfig(v) for v in config_dict.get('uart', [])]
        self.i2c = [I2cConfig(v) for v in config_dict.get('i2c', [])]
        self.spi = [SpiConfig(v) for v in config_dict.get('spi', [])]
        # self.can = [CanConfig(v) for v in config_dict.get('can', [])]

    def compile(self):
        config_buffers = []
        config_buffer_offsets = []
        offset = 0

        for p in self.gpio:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.timer:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.pwm:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.encoder:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.usart:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.i2c:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        for p in self.spi:
            buffer = align_buffer(p.compile())
            config_buffers.append(buffer)
            config_buffer_offsets.append(offset)
            offset += len(buffer)

        # compute 8 bytes aligned start offset
        start_offset = 2 + 2 * len(config_buffers)
        if start_offset % 8:
            start_offset += 8 - (start_offset % 8)
        buff = struct.pack('<H', len(config_buffers))
        for cbo in config_buffer_offsets:
            buff += struct.pack('<H', start_offset + cbo)
        buff += b'\0' * (start_offset - len(buff))
        for cb in config_buffers:
            buff += cb
        return buff

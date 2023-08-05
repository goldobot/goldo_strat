import asyncio
import logging
import serial

SERIAL_PORT = "/dev/ttyPneuma"
BAUDRATE = 115200
RESET = 0x00
CANNON = 0x40
COMPRESSOR = 0x80
VALVES = 0xC0
LED = 0xD0
TURBINE = 0xE0
LCD = 0xF0

LOGGER = logging.getLogger(__name__)

class PneumaticCommands:
    def __init__(self, robot):
        self._robot = robot
        self._pressure_command = 0
        try:
            self._serial = serial.Serial(SERIAL_PORT, BAUDRATE)
            LOGGER.info("initialized serial port for pneumatic board on " + SERIAL_PORT)
        except:
            LOGGER.error("cannot initialize serial port " + SERIAL_PORT)

    def reset(self):
        LOGGER.info("Reset nucleo")
        self._serial.write(bytes([RESET]))

    def reset_motors(self):
        LOGGER.info("Reset moteurs_init DSHOT")
        self._serial.write(b'\x02')

    def stop_compressor(self):
        LOGGER.info("Stop compressor")
        if self._pressure_command > 0x3F:
            self._pressure_command = 0x3F
        _val = RESET | self._pressure_command
        self._serial.write(bytes([_val]))

    def shoot_cannon(self, speed_left, speed_right, speed_top):
        if speed_left > 3:
            speed_left = 3
        if speed_right > 3:
            speed_right = 3
        if speed_top > 3:
            speed_right = 3
        LOGGER.info("Cannon L = " + str(speed_left) + " | R = " + str(speed_right) + " | T = " + str(speed_top))
        _val = CANNON | speed_left << 4 | speed_right << 2 | speed_top
        self._serial.write(bytes([_val]))

    def stop_cannon(self):
        LOGGER.info("Stop cannon")
        self._serial.write(bytes([CANNON]))

    def start_compressor(self, pressure):
        self._pressure_command = pressure
        if self._pressure_command > 0x3F:
            self._pressure_command = 0x3F
        LOGGER.info("Start compressor : pressure command = " + str(0.1 * self._pressure_command) + " bar")
        _val = COMPRESSOR | pressure
        self._serial.write(bytes([_val]))

    def purge_compressor(self):
        LOGGER.info("Purge compressor")
        self._serial.write(bytes([COMPRESSOR]))

    def set_valves(self, a, b, c, e):
        LOGGER.info("Valves : a = " + str(a) + " | b = " + str(b) + " | c = " + str(c) + " | e = " + str(e))
        _val = VALVES | a & 0x08 | b & 0x04 | c & 0x02 | e & 0x01
        self._serial.write(bytes([_val]))

    def start_turbine(self, speed):
        if speed > 3:
            speed = 3
        LOGGER.info("Start turbine, speed = " + str(speed))
        _val = TURBINE | speed
        self._serial.write(bytes([_val]))

    def stop_turbine(self):
        LOGGER.info("Stop turbine")
        self._serial.write(bytes([TURBINE]))

    def led_on(self):
        LOGGER.info("LED ON")
        self._serial.write(bytes([LED | 1]))

    def led_off(self):
        LOGGER.info("LED OFF")
        self._serial.write(bytes([LED | 0]))

    def lcd_on(self):
        LOGGER.info("LCD ON")
        self._serial.write(bytes([LCD | 1]))

    def lcd_off(self):
        LOGGER.info("LCD OFF")
        self._serial.write(bytes([LCD | 0]))

import asyncio
import logging
import serial
import struct

LOGGER = logging.getLogger(__name__)

SERIAL_PORT = "/dev/goldorak/ttyPneuma"
BAUDRATE = 115200


class MsgType:
	UNKNOWN_MSG_TYPE = 0
	TELEMETRY_MSG = 1
	ERROR_MSG = 2
	COMMAND_MSG = 3
	MAX_MSG = 4

class TlmMsg:
	UNKNOWN_TLM = 0
	COMP_TLM = 1
	MAX_TLM = 2

class ErrorMsg:
	UNKNOWN_ERROR = 0
	UNKNOWN_MSG = 1
	WRONG_HEADER = 2
	MSG_TOO_SHORT = 3
	WRONG_CRC = 4
	COMPRESSOR_OVERHEATING = 5
	COMPRESSOR_OVERPRESSURE = 6
	MAX_ERROR = 7

	errorStrings = {
		UNKNOWN_ERROR: "Unknown error",
		UNKNOWN_MSG: "Unknown message",
		WRONG_HEADER: "Wrong header",
		MSG_TOO_SHORT: "Message too short",
		WRONG_CRC: "Wrong CRC",
		COMPRESSOR_OVERHEATING: "Compressor overheating",
		COMPRESSOR_OVERPRESSURE: "Compressor overpressure"
	}

class CmdMsg:
	UNKNOWN_CMD = 0
	COMP_ON_CMD = 1
	COMP_OFF_CMD = 2
	PURGE_CMD = 3
	VALVE_ON_CMD = 4
	VALVE_OFF_CMD = 5
	MULTIPLE_VALVES_ON_CMD = 6
	MULTIPLE_VALVES_OFF_CMD = 7
	PUMP_ON_CMD = 8
	PUMP_OFF_CMD = 9
	MAX_CMD = 10

LOGGER = logging.getLogger(__name__)

class PneumaticCommands:
	HEADER = b'\x42'
	pressure = 0
	temperature = 0

	def __init__(self, robot):
		self._robot = robot
		self._connect_serial()

	def _connect_serial(self):
		try:
			self._serial = serial.Serial(SERIAL_PORT, BAUDRATE)
			LOGGER.info("initialized serial port for pneumatic board on " + SERIAL_PORT)
		except:
			LOGGER.error("cannot initialize serial port " + SERIAL_PORT)

	def _compute_crc8(self, data: bytes) -> int:
		crc = 0
		for b in data:
			crc ^= b
			for _ in range(8):
				if crc & 0x80:
					crc = (crc << 1) ^ 0x07
				else:
					crc <<= 1
				crc &= 0xFF
		return crc

	async def _send_command_message(self, *args):
		if len(args) == 0 :
			raise ValueError("No command provided")

		# args[0] is the command
		frame = self.HEADER + bytes([MsgType.COMMAND_MSG]) + bytes([args[0]])

		# if there is a second argument, it is a value to send
		if len(args) == 2 :
			frame = frame + bytes([args[1]])
		elif len(args) > 2 :
			frame = frame + bytes([len(args) - 1])
			for arg in args[1:] :
				frame = frame + bytes([arg])

		crc = self._compute_crc8(frame)
		try:
			self._serial.write(frame + bytes([crc]))
		except IOError:
			LOGGER.error("IO error while sending command to pneumatic board")
			self._connect_serial()
			self._serial.write(frame + bytes([crc]))

	# Turns the compressor on to the specified pressure (in tenth of bar)
	async def compressor_on(self, pressure):
		await self._send_command_message(CmdMsg.COMP_ON_CMD, pressure)
		LOGGER.info("compressor on with pressure " + str(pressure))

	# Turns the compressor off
	async def compressor_off(self):
		await self._send_command_message(CmdMsg.COMP_OFF_CMD)
		LOGGER.info("compressor off")

	# Turns the compressor off and opens the purge valve until the pressure is below 0.5 bar
	async def purge(self):
		await self._send_command_message(CmdMsg.PURGE_CMD)
		LOGGER.info("purge")

	# Turns the specified valve on
	async def valve_on(self, valve_id):
		await self._send_command_message(CmdMsg.VALVE_ON_CMD, valve_id)
		LOGGER.info("valve " + str(valve_id) + " on")

	# Turns the specified valve off
	async def valve_off(self, valve_id):
		await self._send_command_message(CmdMsg.VALVE_OFF_CMD, valve_id)
		LOGGER.info("valve " + str(valve_id) + " off")

	async def multiple_valves_on(self, *valves):
		await self._send_command_message(CmdMsg.MULTIPLE_VALVES_ON_CMD, *valves)
		LOGGER.info("valves " + str(valves) + " on")

	async def _read_message(self):
		try:
			msgtype = 0
			if self._serial.read(1) == self.HEADER :
				msgtype = self._serial.read(1)
				message = self._serial.read(1)
				if msgtype == MsgType.TELEMETRY_MSG :
					if message == TlmMsg.COMP_TLM :
						payload = self._serial.read(2)
					crc = self._serial.read(1)[0]
					if self._compute_crc8(self.HEADER + MsgType.TELEMETRY_MSG + TlmMsg.COMP_TLM + payload) == crc:
						self.pressure, self.temperature = struct.unpack('<BB', payload)
						LOGGER.info(f"Pressure: {self.pressure}, Temperature: {self.temperature}")
					else:
						LOGGER.error("CRC mismatch")
				elif msgtype == MsgType.ERROR_MSG :
					LOGGER.error("Error : " + ErrorMsg.errorStrings[message])
		except IOError:
			LOGGER.error("IO error while reading message from pneumatic board")
			self._connect_serial()

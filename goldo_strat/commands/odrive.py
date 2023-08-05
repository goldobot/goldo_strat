import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import functools
import math
import logging

import numpy as np
import scipy.interpolate

from typing import Mapping

LOGGER = logging.getLogger(__name__)


class ODriveCommands:
    _sequence_number: int
    _futures: Mapping[int, asyncio.Future]

    _broker: object
    _loop: object

    def __init__(self, robot):
        self._robot = robot
        self._sequence_number = 1
        self._loop = asyncio.get_event_loop()
        self._futures = {}
        self._commands = {}
        self._protocol_version = 0x9b40
        self._robot._broker.registerCallback('nucleo/out/propulsion/odrive/axis_states', self._onODriveAxisStates)
        self._robot._broker.registerCallback('nucleo/out/propulsion/odrive/errors', self._onODriveAxisErrors)

    def setBroker(self, broker):
        self._broker = broker
        self._broker.registerCallback('nucleo/out/propulsion/cmd_event', self._on_cmd_event)

    @property
    def error(self):
        proto = self._robot._state_proto.nucleo.odrive
        return proto.axis0.errors.axis != 0 or proto.axis1.errors.axis

    async def clearErrors(self):
        await self._sendRequest(228 + 71, 0, b'')
        await self._sendRequest(228 + 300, 0, b'')

    async def _sendRequest(self, endpoint_id, expected_response_size, payload):
        seq = 0
        msg = _sym_db.GetSymbol('goldo.nucleo.odrive.RequestPacket')()
        msg.sequence_number = seq
        msg.endpoint_id = endpoint_id
        msg.expected_response_size = expected_response_size
        msg.payload = payload
        msg.protocol_version = self._protocol_version
        await self._robot._broker.publishTopic('nucleo/in/odrive/request', msg)
        return seq

    def _publish(self, topic, msg=None):
        return self._robot._broker.publishTopic(topic, msg)

    async def _onODriveAxisErrors(self, msg):
        proto = self._robot._state_proto.nucleo.odrive
        proto.axis0.errors.CopyFrom(msg.axis0)
        proto.axis1.errors.CopyFrom(msg.axis1)

    async def _onODriveAxisStates(self, msg):
        proto = self._robot._state_proto.nucleo.odrive

        proto.axis0.current_state = msg.axis0.current_state
        proto.axis0.requested_state = msg.axis0.requested_state
        proto.axis0.control_mode = msg.axis0.control_mode

        proto.axis1.current_state = msg.axis1.current_state
        proto.axis1.requested_state = msg.axis1.requested_state
        proto.axis1.control_mode = msg.axis1.control_mode

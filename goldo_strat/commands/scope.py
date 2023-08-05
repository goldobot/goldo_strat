import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import functools
import math
import logging
import struct
import numpy as np

from typing import Mapping

LOGGER = logging.getLogger(__name__)


def _variableSize(encoding):
    if encoding == 4:
        return 1
    if encoding == 5:
        return 2
    if encoding == 6:
        return 4


def _variableFactor(encoding):
    if encoding == 4:
        return 1 / ((1 << 8) - 1)
    if encoding == 5:
        return 1 / ((1 << 16) - 1)
    if encoding == 6:
        return 1 / ((1 << 32) - 1)


def _variableCode(encoding):
    if encoding == 4:
        return 'B'
    if encoding == 5:
        return 'H'
    if encoding == 6:
        return 'I'


propulsion_variables = [
    '',
    'pose.x',
    'pose.y',
    'pose.yaw',
    'pose.speed',
    'pose.yaw_rate',
    'target.x',
    'target.y',
    'target.yaw',
    'target.speed',
    'target.yaw_rate',
    'low_level.longi_error',
    'low_lewel.yaw_error',
    'low_level.speed_error',
    'low_level.yaw_rate_error',
    'motor0.vel_setpoint',
    'motor1.vel_setpoint',
    'odrive.axis0.vel_estimate',
    'odrive.axis1.vel_estimate',
    'odrive.axis0.current_iq_setpoint',
    'odrive.axis1.current_iq_setpoint',
]

propulsion_variables_dict = {v: i for i, v in enumerate(propulsion_variables)}


class ScopeCommands:
    _broker: object

    def __init__(self):
        self._config = None
        self._total_size = 1
        self._ref_timestamp = 0
        self._channels = [_sym_db.GetSymbol('goldo.nucleo.ScopeChannelConfig')() for i in range(8)]
        self._channel_values = []
        self._channel_timestamps = []
        self._max_points = 1000
        self._running = False

    async def setChannel(self, num, variable, min_value, max_value):
        self._channels[num].variable = propulsion_variables_dict[variable]
        self._channels[num].encoding = 4
        self._channels[num].min_value = min_value
        self._channels[num].max_value = max_value
        await self._updateConfig()

    async def configTimebase(self, period):
        self._config.period = period
        await self._updateConfig()

    def start(self):
        self._running = True
        self._channel_timestamps = [np.array([], dtype=np.float32) for i in range(len(self._config.channels))]
        self._channel_values = [np.array([], dtype=np.float32) for i in range(len(self._config.channels))]

    def stop(self):
        self._running = False

    def saveToCsv(self):
        data = []
        for i in range(len(self._config.channels)):
            data.append(self._channel_values[i])
        data = np.array(data).T  # transpose the array to have proper columns
        np.savetxt('columns_from_np_arrays.csv', data, delimiter=',')

    async def _updateConfig(self):
        msg = _sym_db.GetSymbol('goldo.nucleo.ScopeConfig')(period=10)

        for channel in self._channels:
            if channel.variable != 0:
                msg.channels.append(channel)
        await self._broker.publishTopic('nucleo/in/propulsion/scope/config/set', msg)
        await self.onConfig(msg)

    def setBroker(self, broker):
        self._broker = broker
        self._broker.registerCallback('nucleo/in/propulsion/scope/config/set', self.onConfig)
        self._broker.registerCallback('nucleo/out/propulsion/scope/data', self.onData)
        self._broker.registerCallback('nucleo/out/os/heartbeat', self.onHeartBeat)

    async def onHeartBeat(self, msg):
        self._ref_timestamp = msg.timestamp

    async def onConfig(self, msg):
        self._config = msg
        self._total_size = sum([_variableSize(chan.encoding) for chan in self._config.channels])
        self._factors = [_variableFactor(chan.encoding) for chan in self._config.channels]
        self._struct = struct.Struct('<' + ''.join([_variableCode(chan.encoding) for chan in self._config.channels]))
        self._channel_timestamps = [np.array([], dtype=np.float32) for i in range(len(self._config.channels))]
        self._channel_values = [np.array([], dtype=np.float32) for i in range(len(self._config.channels))]

    async def onData(self, msg):
        if self._config is None:
            return
        timestamp_delta = (msg.timestamp - self._ref_timestamp) % (1 << 16)
        if timestamp_delta > (1 << 15):
            timestamp_delta -= (1 << 16)
        timestamp_base = self._ref_timestamp + timestamp_delta
        out_msg = _sym_db.GetSymbol('goldo.nucleo.ScopeValues')()
        out_msg.channels.extend(
            [_sym_db.GetSymbol('goldo.nucleo.ScopeChannelValues')() for i in range(len(self._config.channels))])

        for i in range(len(msg.data) // self._total_size):
            out_msg.timestamps.append(1e-3 * (timestamp_base + self._config.period * i))
            vals = self._struct.unpack(msg.data[i * self._total_size:(i + 1) * self._total_size])
            for j in range(len(self._config.channels)):
                chan = self._config.channels[j]
                val = vals[j] * self._factors[j]
                val = val * (chan.max_value - chan.min_value) + chan.min_value
                out_msg.channels[j].float_values.append(val)

        if self._running:
            for i, channel in enumerate(out_msg.channels):
                if i < len(self._channel_values):
                    self._channel_values[i] = np.append(self._channel_values[i], channel.float_values)
                    self._channel_timestamps[i] = np.append(self._channel_timestamps[i], out_msg.timestamps)
                    if self._channel_values[i].shape[0] > self._max_points:
                        self._channel_timestamps[i] = self._channel_timestamps[i][-self._max_points:]
                        self._channel_values[i] = self._channel_values[i][-self._max_points:]

        await self._broker.publishTopic('main/propulsion/scope/values', out_msg)

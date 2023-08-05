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


class PropulsionCommand(object):
    def __init__(self, loop, sequence_number):
        self._sequence_number = sequence_number
        self._status = -1
        self._loop = loop
        self._future_begin = None
        self._future_end = None

    @property
    def begin(self):
        if self._future_begin is None:
            self._future_begin = self._loop.create_future()
            self._update_status()
        return self._future_begin

    @property
    def end(self):
        if self._future_end is None:
            self._future_end = self._loop.create_future()
            self._update_status()
        return self._future_end

    def _update_status(self, status=None):
        if status is not None:
            self._status = status
        if self._status == 0 and self._future_begin and not self._future_begin.done():
            self._future_begin.set_result(None)
        if self._status == 1 and self._future_end and not self._future_end.done():
            self._future_end.set_result(None)


class PropulsionCommands:
    _sequence_number: int
    _futures: Mapping[int, asyncio.Future]

    _broker: object
    _loop: object

    def __init__(self):
        self._sequence_number = 1
        self._loop = asyncio.get_event_loop()
        self._futures = {}
        self._commands = {}

    def setBroker(self, broker):
        self._broker = broker
        self._broker.registerCallback('nucleo/out/propulsion/cmd_event', self._on_cmd_event)

    def _create_future(self):
        future = self._loop.create_future()
        sequence_number = self._sequence_number
        self._sequence_number = (self._sequence_number + 1) % (1 << 15)
        LOGGER.debug("PropulsionCommands: create future, sequence_number=%s", sequence_number)

        self._futures[sequence_number] = future
        future.add_done_callback(functools.partial(self._remove_future, sequence_number))
        return sequence_number, future
        cmd = PropulsionCommand(self._loop, sequence_number)
        self._commands[sequence_number] = cmd
        return sequence_number, cmd

    def _create_command_msg(self, name):
        sequence_number, future = self._create_future()
        msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.' + name)()
        msg.sequence_number = sequence_number
        return msg, future

    def _remove_future(self, sequence_number: int, future: asyncio.Future):
        LOGGER.debug("Remove done future %r", sequence_number)
        self._futures.pop(sequence_number, None)

    def _publish(self, topic, msg=None):
        return self._broker.publishTopic(topic, msg)

    def setEnable(self, enable):
        return self._publish('nucleo/in/propulsion/enable/set',
                             _sym_db.GetSymbol('google.protobuf.BoolValue')(value=enable))

    def setMotorsEnable(self, enable):
        return self._publish('nucleo/in/propulsion/motors/enable/set',
                             _sym_db.GetSymbol('google.protobuf.BoolValue')(value=enable))

    def setAccelerationLimits(self, accel, deccel, angular_accel, angular_deccel):
        msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.AccelerationLimits')()
        msg.accel = accel
        msg.deccel = deccel
        msg.angular_accel = angular_accel
        msg.angular_deccel = angular_deccel
        return self._publish('nucleo/in/propulsion/motors/acceleration_limits/set', msg)

    def clearCommandQueue(self):
        return self._publish('nucleo/in/propulsion/clear_command_queue')

    async def setTargetSpeed(self, target_speed):
        msg, future = self._create_command_msg('ExecuteSetTargetSpeed')
        msg.target_speed = target_speed
        await self._publish('nucleo/in/propulsion/target_speed/set', msg)
        await future

    async def setPose(self, pt, yaw):
        msg = _sym_db.GetSymbol('goldo.common.geometry.Pose')()
        msg.yaw = yaw * math.pi / 180
        msg.position.x = pt[0]
        msg.position.y = pt[1]
        await self._publish('nucleo/in/propulsion/pose/set', msg)

    async def translation(self, distance, speed):
        msg, future = self._create_command_msg('ExecuteTranslation')
        msg.distance = distance
        msg.speed = speed
        await self._publish('nucleo/in/propulsion/cmd/translation', msg)
        await future

    async def moveTo(self, pt, speed):
        msg, future = self._create_command_msg('ExecuteMoveTo')
        msg.speed = speed
        msg.point.x = pt[0]
        msg.point.y = pt[1]
        await self._publish('nucleo/in/propulsion/cmd/move_to', msg)
        await future

    async def rotation(self, angle, yaw_rate):
        msg, future = self._create_command_msg('ExecuteRotation')
        msg.angle = angle * math.pi / 180
        msg.yaw_rate = yaw_rate

        await self._publish('nucleo/in/propulsion/cmd/rotation', msg)
        await future

    async def pointTo(self, pt, yaw_rate):
        msg, future = self._create_command_msg('ExecutePointTo')
        msg.yaw_rate = yaw_rate
        msg.point.x = pt[0]
        msg.point.y = pt[1]

        await self._publish('nucleo/in/propulsion/cmd/point_to', msg)
        await future

    async def faceDirection(self, yaw, yaw_rate):
        msg, future = self._create_command_msg('ExecuteFaceDirection')
        msg.yaw_rate = yaw_rate
        msg.yaw = yaw * math.pi / 180

        await self._publish('nucleo/in/propulsion/cmd/face_direction', msg)
        await future

    async def trajectory(self, points, speed):
        msg, future = self._create_command_msg('ExecuteTrajectory')
        msg.speed = speed
        Point = _sym_db.GetSymbol('goldo.common.geometry.Point')
        msg.points.extend([Point(x=pt[0], y=pt[1]) for pt in points])

        await self._publish('nucleo/in/propulsion/cmd/trajectory', msg)
        await future

    async def trajectorySpline(self, points, speed):
        ctr = np.array([points[0]] + points + [points[-1]])

        # control points, double first and last
        x = ctr[:, 0]
        y = ctr[:, 1]

        # knots
        l = len(x)
        t = np.linspace(0, 1, l - 2, endpoint=True)
        t = np.append([0, 0, 0], t)
        t = np.append(t, [1, 1, 1])

        tck = [t, [x, y], 3]

        num_samples = 16

        u3 = np.linspace(0, 1, num_samples, endpoint=True)
        out = scipy.interpolate.splev(u3, tck)
        sampled_points = [(out[0][i], out[1][i]) for i in range(num_samples)]

        await self.trajectory(sampled_points, speed)

    async def setPose(self, pt, yaw):
        msg = _sym_db.GetSymbol('goldo.common.geometry.Pose')()
        msg.yaw = yaw * math.pi / 180
        msg.position.x = pt[0]
        msg.position.y = pt[1]
        await self._publish('nucleo/in/propulsion/pose/set', msg)

    async def _on_cmd_event(self, msg):
        future = self._futures.get(msg.sequence_number)
        if future is not None:
            if msg.status == 1:
                future.set_result(None)
                return

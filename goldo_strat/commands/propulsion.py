__all__ = [
    'PropulsionCommands',
    'PropulsionError']

import struct

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

#import RPi.GPIO as GPIO

import mmap
import os
import time


LOGGER = logging.getLogger(__name__)


class PropulsionError(Exception):
    def __init__(self, error):
        self.error = error


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

    def __init__(self, robot):
        self._robot = robot
        self._sequence_number = 1
        self._loop = asyncio.get_event_loop()
        self._futures = {}
        self._futures_ack = {}
        self._commands = {}
        self._cmd = {}
        self._sensor_ids = {}
        self._broker = self._robot._broker
        self._broker.registerCallback('nucleo/out/propulsion/telemetry', self._onTelemetryMsg)
        self._broker.registerCallback('nucleo/out/propulsion/cmd_event', self._on_cmd_event)
        self._broker.registerCallback('nucleo/out/propulsion/controller/event', self._on_controller_event)
        self._broker.registerCallback('gui/in/robot_state', self._on_robot_state)
        self.state = 0

        self.adversary_detection_enable = True
        # special front adversary detection for arms
        self.adversary_detection_front_distance = 0
        self.adversary_detection_side_distance = 0
        
        self.speed = 1
        self.yaw_rate = 4

        self.rplidar_shmem_fd = open("/home/goldorak/workspace/goldo_main/rplidar_shmem.txt","a+b")
        self.rp_shmem = mmap.mmap(self.rplidar_shmem_fd.fileno(), 4096, access=mmap.ACCESS_WRITE, offset=0)

        # FIXME : TODO
        self.current_command = "u"

    def loadConfig(self):
        self._sensor_ids = {}
        for i, sensor_proto in enumerate(self._robot._config_proto.nucleo.sensors):
            name = sensor_proto.name
            self._sensor_ids[name] = i

    def _create_future(self):
        future = self._loop.create_future()
        sequence_number = self._sequence_number
        self._sequence_number = (self._sequence_number + 1) % (1 << 15)
        LOGGER.debug("PropulsionCommands: create future, sequence_number=%s", sequence_number)
        self._futures[sequence_number] = future
        future.add_done_callback(functools.partial(self._remove_future, sequence_number))
        return sequence_number, future

    def _create_command_msg(self, name, is_cmd=False):
        sequence_number, future = self._create_future()
        msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.' + name)()
        msg.sequence_number = sequence_number
        self._cmd[sequence_number] = is_cmd
        return msg, future

    def _remove_future(self, sequence_number: int, future: asyncio.Future):
        LOGGER.debug("Remove done future %r", sequence_number)
        self._futures.pop(sequence_number, None)

    def _remove_future_ack(self, sequence_number: int, future: asyncio.Future):
        LOGGER.debug("Remove done future ack %r", sequence_number)
        self._futures_ack.pop(sequence_number, None)

    def _publish(self, topic, msg=None):
        LOGGER.debug("Propulsion publish %s %s", topic, msg)
        return self._broker.publishTopic(topic, msg)

    async def _publish_sequence(self, topic, msg):
        LOGGER.debug('propulsion cmd msg %s', msg.sequence_number)
        future_ack = self._loop.create_future()
        self._futures_ack[msg.sequence_number] = future_ack
        future_ack.add_done_callback(functools.partial(self._remove_future_ack, msg.sequence_number))

        await self._publish(topic, msg)

        try:
            await asyncio.wait_for(future_ack, 5)
            return
        except asyncio.TimeoutError:
            LOGGER.error('propulsion timeout on command %s, retry', msg)

        try:
            await asyncio.wait_for(future_ack, 5)
        except asyncio.TimeoutError:
            LOGGER.error('propulsion timeout on command %s', msg)

    @property
    def pose(self):
        return self._robot._state_proto.robot_pose

    async def setEnable(self, enable):
        msg, future = self._create_command_msg('CmdSetEnable')
        msg.enable = enable
        await self._publish_sequence('nucleo/in/propulsion/enable/set', msg)
        await future

    def setMotorsEnable(self, enable):
        return self._publish('nucleo/in/propulsion/motors/enable/set',
                             _sym_db.GetSymbol('google.protobuf.BoolValue')(value=enable))

    async def setAccelerationLimits(self, accel, deccel, angular_accel, angular_deccel):
        msg, future = self._create_command_msg('CmdSetAccelerationLimits')
        msg.accel = accel
        msg.deccel = deccel
        msg.angular_accel = angular_accel
        msg.angular_deccel = angular_deccel
        await self._publish_sequence('nucleo/in/propulsion/acceleration_limits/set', msg)
        await future

    def setMotorsTorqueLimits(self, left, right):
        sequence_number = self._sequence_number
        self._sequence_number = (self._sequence_number + 1) % (1 << 15)

        msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.SetMotorsTorqueLimits')()
        msg.sequence_number = sequence_number
        msg.left = left
        msg.right = right
        return self._publish('nucleo/in/propulsion/motors/torque_limits/set', msg)

    def clearCommandQueue(self):
        return self._publish('nucleo/in/propulsion/clear_command_queue')

    async def setTargetSpeed(self, target_speed):
        msg, future = self._create_command_msg('CmdSetTargetSpeed')
        msg.target_speed = target_speed
        await self._publish('nucleo/in/propulsion/target_speed/set', msg)
        await future

    async def setPose(self, pt, yaw):
        msg, future = self._create_command_msg('CmdSetPose')
        msg.yaw = yaw * math.pi / 180
        msg.position.x = pt[0]
        msg.position.y = pt[1]
        await self._publish_sequence('nucleo/in/propulsion/pose/set', msg)
        await future

    async def transformPose(self, translation, rotation, angle_unit='deg'):
        msg, future = self._create_command_msg('CmdTransformPose')
        msg.rotation = rotation * (math.pi / 180 if angle_unit == 'deg' else 1)
        msg.translation.x = translation[0]
        msg.translation.y = translation[1]
        await self._publish_sequence('nucleo/in/propulsion/pose/transform', msg)
        await future

    async def emergencyStop(self):
        msg, future = self._create_command_msg('CmdEmpty')
        await self._publish_sequence('nucleo/in/propulsion/emergency_stop', msg)
        await future

    async def clearError(self):
        msg, future = self._create_command_msg('CmdEmpty')
        await self._publish_sequence('nucleo/in/propulsion/clear_error', msg)
        await future

    async def translation(self, distance, speed):
        LOGGER.debug("********************************************")
        LOGGER.debug(" translation({:f}, {:f})".format(distance, speed))
        LOGGER.debug("********************************************")
        msg, future = self._create_command_msg('ExecuteTranslation', True)
        msg.distance = distance
        msg.speed = speed
        await self._publish_sequence('nucleo/in/propulsion/cmd/translation', msg)
        await future

    async def reposition(self, distance, speed):
        msg, future = self._create_command_msg('ExecuteReposition', True)
        msg.distance = distance
        msg.speed = speed
        self._reposition_event = None
        await self._publish_sequence('nucleo/in/propulsion/cmd/reposition', msg)
        await future
        return self._reposition_event

    async def measureNormal(self, angle, distance):
        msg, future = self._create_command_msg('CmdMeasureNormal')
        msg.angle = angle * math.pi / 180
        msg.distance = distance
        await self._publish_sequence('nucleo/in/propulsion/cmd/measure_normal', msg)
        await future

    async def moveTo(self, pt, speed):
        msg, future = self._create_command_msg('ExecuteMoveTo', True)
        msg.speed = speed
        msg.point.x = pt[0]
        msg.point.y = pt[1]
        await self._publish_sequence('nucleo/in/propulsion/cmd/move_to', msg)
        await future

    async def moveToRetry(self, p, speed, retreat_dist=-0.15, retries=10, eternal_sleep=True, retry_speed_factor=1.0):
        LOGGER.debug("********************************************")
        LOGGER.debug("**                                        **")
        LOGGER.debug("** moveToRetry()                          **")
        LOGGER.debug("**                                        **")
        LOGGER.debug("********************************************")
        cp0_x = self.pose.position.x
        cp0_y = self.pose.position.y
        s = speed
        retreat_dist=abs(retreat_dist)
        for i in range(0,retries):
            try:
                LOGGER.debug("********************************************")
                LOGGER.debug("    # {:d}".format(i))
                LOGGER.debug("********************************************")
                await self.moveTo(p, s)
                await asyncio.sleep(0.5)
                LOGGER.debug("  moveToRetry() OK")
                return
            except PropulsionError as e:
                LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                LOGGER.debug("    EXCEPTION !")
                LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                LOGGER.debug("  DISABLE DETECTION")
                self.adversary_detection_enable = False
                self.rp_shmem[0] = 0x00
                await asyncio.sleep(0.2)
                LOGGER.debug("  PROPULSION DISABLE/ENABLE (to clear error)")
                self.setEnable(False)
                await asyncio.sleep(0.2)
                self.setEnable(True)
                await asyncio.sleep(0.2)
                self.setMotorsEnable(True)
                await asyncio.sleep(0.2)
                LOGGER.debug("  re-clear error")
                await self.clearError()
                await asyncio.sleep(1.0)

                LOGGER.debug("  COMPUTE RETREAT DISTANCE")
                cp_x = self.pose.position.x
                cp_y = self.pose.position.y
                cp_yaw = self.pose.yaw
                #LOGGER.debug ("cp0 = ({:f}, {:f})".format(cp0_x, cp0_y))
                #LOGGER.debug ("cp = ({:f}, {:f})".format(cp_x, cp_y))
                dx = cp_x - cp0_x
                dy = cp_y - cp0_y
                #LOGGER.debug ("delta = ({:f}, {:f})".format(dx, dy))
                max_dist = np.sqrt(dx*dx+dy*dy)
                #LOGGER.debug ("max_dist = {:f}".format(max_dist))
                if (max_dist>retreat_dist):
                    max_dist = retreat_dist
                if ((dx*np.cos(cp_yaw)+dy*np.sin(cp_yaw))>0):
                    escape_dist = -max_dist
                else:
                    escape_dist = max_dist
                LOGGER.debug("  RETREAT {:f} mm".format(escape_dist))
                await self.translation(escape_dist, s)

            LOGGER.debug("  CHECK ADVERSARY PRESENCE")
            while True:
                free_path = True
                rplidar = self._robot._state_proto.rplidar
                for det in rplidar.detections:
                    dx = det.x - cp_x
                    dy = det.y - cp_y
                    obst_dist = np.sqrt(dx*dx+dy*dy)
                    if (obst_dist<0.3) : free_path = False
                if free_path:
                    break
                await asyncio.sleep(0.5)
                # FIXME : TODO : what to do if dumb adversary? (which frozes after contact)

            LOGGER.debug("  no near adversary -> REENABLE DETECTION")
            self.adversary_detection_enable = True
            self.rp_shmem[0] = 0x01
            await asyncio.sleep(1)

            LOGGER.debug("  RETRY MOVE with lower speed")
            s = speed * retry_speed_factor

        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        LOGGER.debug("!!                                        !!")
        LOGGER.debug("!! moveToRetry() failed                   !!")
        LOGGER.debug("!!                                        !!")
        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if (eternal_sleep):
            LOGGER.debug("sleep eternally....")
            await asyncio.sleep(3600.0)

    async def rotation(self, angle, yaw_rate=None):
        if yaw_rate is None:
            yaw_rate = self.yaw_rate
        msg, future = self._create_command_msg('ExecuteRotation', True)
        msg.angle = angle * math.pi / 180
        msg.yaw_rate = yaw_rate

        await self._publish_sequence('nucleo/in/propulsion/cmd/rotation', msg)
        await future

    async def pointTo(self, pt, yaw_rate=None, back=False):
        if yaw_rate is None:
            yaw_rate = self.yaw_rate
        msg, future = self._create_command_msg('ExecutePointTo', True)
        msg.yaw_rate = yaw_rate
        msg.point.x = pt[0]
        msg.point.y = pt[1]

        if back:
            await self._publish_sequence('nucleo/in/propulsion/cmd/point_to_back', msg)
        else:
            await self._publish_sequence('nucleo/in/propulsion/cmd/point_to', msg)
        await future

    async def pointAndGo(self, pt, speed, yaw_rate, back=False):
        await self.pointTo(pt, yaw_rate, back)
        await self.moveTo(pt, speed)

    async def faceDirection(self, yaw, yaw_rate = None):
        if yaw_rate is None:
            yaw_rate = self.yaw_rate
        msg, future = self._create_command_msg('ExecuteFaceDirection', True)
        msg.yaw_rate = yaw_rate
        msg.yaw = yaw * math.pi / 180

        await self._publish_sequence('nucleo/in/propulsion/cmd/face_direction', msg)
        await future

    async def trajectory(self, points, speed, *, reposition_distance=0, reposition_speed=0):
        msg, future = self._create_command_msg('ExecuteTrajectory', True)
        msg.speed = speed
        msg.reposition_distance = reposition_distance
        msg.reposition_speed = reposition_speed
        Point = _sym_db.GetSymbol('goldo.common.geometry.Point')
        msg.points.extend([Point(x=pt[0], y=pt[1]) for pt in points])

        await self._publish_sequence('nucleo/in/propulsion/cmd/trajectory', msg)
        await future

    async def trajectorySpline(self, points, speed = None, **kwargs):
        if speed is None:
            speed = self.speed
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
        return await self.trajectory(sampled_points, speed, **kwargs)

    async def _setEventSensorsMaks(self, sensors):
        msg, future = self._create_command_msg('CmdSetEventSensorsMask')
        rising_mask = 0
        falling_mask = 0
        for k, v in sensors.items():
            if v in ['rising', 'both']:
                rising_mask |= 1 << self._sensor_ids[k]
            if v in ['falling', 'both']:
                falling_mask |= 1 << self._sensor_ids[k]

        msg.mask_rising = rising_mask
        msg.mask_falling = falling_mask

        await self._publish_sequence('nucleo/in/propulsion/set_event_sensors_mask', msg)
        await future

    async def _adversary_detection(self):
        #basic adversary detection
        robot_pose = self._robot._state_proto.robot_pose
        rplidar = self._robot._state_proto.rplidar

        adversary_detected = False

        if robot_pose.speed > 0 and rplidar.zones.front_near:
            adversary_detected = True
        if robot_pose.speed < 0 and rplidar.zones.back_near:
            adversary_detected = True
        if robot_pose.speed > 0.5 and rplidar.zones.front_far:
            adversary_detected = True
        if robot_pose.speed < -0.5 and rplidar.zones.back_far:
            adversary_detected = True

        if robot_pose.speed > 0:
            # rotation matrix and translation vector
            p_ref = np.array([robot_pose.position.x, robot_pose.position.y])
            yaw = robot_pose.yaw
            rot_m = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])

            near_distance = 0.1
            far_distance = self.adversary_detection_front_distance
            if far_distance > near_distance:
                for det in rplidar.detections:
                    p_d = np.array([det.x, det.y])
                    rel = rot_m.dot(p_d - p_ref)

                    if near_distance < rel[0] <= far_distance:
                        u = (rel[0] - near_distance) / (far_distance - near_distance)
                        width = u * 0.2 * (1 - u) * 0.1
                        if abs(rel[1]) * 2 <= width:
                            adversary_detected = True
        if adversary_detected:
            LOGGER.info("********************************************")
            LOGGER.info("********************************************")
            LOGGER.info("*  PropulsionCommands: adversary detected  *")
            LOGGER.info("********************************************")
            LOGGER.info("********************************************")
            #await self.emergencyStop()
            #GPIO.setmode(GPIO.BCM)
            #GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)
            #GPIO.output(21, GPIO.HIGH)
            #await asyncio.sleep(0.0)
            #GPIO.output(21, GPIO.LOW)

    async def _on_robot_state(self, msg):
        #adversary detection
        if self.state == 2 and self.adversary_detection_enable:
            await self._adversary_detection()
        if self.adversary_detection_enable:
            self.rp_shmem[0] = 0x01
        else:
            self.rp_shmem[0] = 0x00
        robot_pose = self._robot._state_proto.robot_pose
        # FIXME : TODO
        self.current_command = "u"
        self.rp_shmem[1:2] = bytearray(self.current_command,"utf8")
        self.rp_shmem[4:8] = struct.pack("<f",robot_pose.speed)

    async def _onTelemetryMsg(self, msg):
        self._robot._state_proto.robot_pose.CopyFrom(msg.pose)
        self.state = msg.state

    async def _on_controller_event(self, msg):
        # reposition
        if msg.type == 1:
            self._reposition_event = msg

    async def _on_cmd_event(self, msg):
        LOGGER.debug('propulsion _on_cmd_event %s', msg)
        if msg.status == 4:
            LOGGER.debug('propulsion cmd ack (seq=%d)', msg.sequence_number)
            future = self._futures_ack.get(msg.sequence_number)
            if future is not None:
                future.set_result(None)
            else:
                LOGGER.debug("future not found")

        future = self._futures.get(msg.sequence_number)

        if future is not None:
            if msg.status == 1:
                future.set_result(None)
                return
            if msg.status == 2:
                future.set_exception(PropulsionError(msg.error))
                return
            if msg.status == 3:
                future.set_exception(PropulsionError(msg.error))
                return
            if msg.status == 4 and not self._cmd.get(msg.sequence_number):
                future.set_result(None)
                return


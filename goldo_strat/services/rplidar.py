import asyncio
from datetime import datetime, timedelta
import math

import logging

import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()

LOGGER = logging.getLogger(__name__)


class RPLidarUpdater:
    def __init__(self, robot):
        self._robot = robot
        self._sensors_map = {}
        self._robot._broker.registerCallback('rplidar/out/detections', self.onDetectionsMsg)
        self._robot._broker.registerCallback('rplidar/out/robot_detection', self.onRobotDetectionMsg)
        self._robot._broker.registerCallback('rplidar/out/emergency_raise', self.onEmergencyRaiseMsg)

        self._last_message_ts = datetime.now() - timedelta(seconds=10)
        self._detections_ts = {}
        self._watchdog_task = asyncio.create_task(self.runWatchdog())

    async def loadConfig(self):
        config_proto = self._robot._config_proto
        msg = _sym_db.GetSymbol('google.protobuf.FloatValue')(
            value=config_proto.rplidar.theta_offset * math.pi / 180)
        await self._robot._broker.publishTopic('rplidar/in/config/theta_offset', msg)
        await self._robot._broker.publishTopic('rplidar/in/config/distance_tresholds', config_proto.rplidar.tresholds)


    async def runWatchdog(self):
        while True:
            await asyncio.sleep(0.5)
            #await asyncio.sleep(2.0)
            #print ("==========")
            self._robot._state_proto.rplidar.running = bool(datetime.now() - self._last_message_ts < timedelta(0.5))
            new_detections = []
            state_proto = self._robot._state_proto
            for d in state_proto.rplidar_detections:
                #print (d)
                if d.detect_quality > 0:
                    d.detect_quality = d.detect_quality - 1
                    new_detections.append(d)
            del state_proto.rplidar_detections[:]
            state_proto.rplidar_detections.extend(new_detections)
            del state_proto.rplidar.detections[:]
            state_proto.rplidar.detections.extend(state_proto.rplidar_detections)

    async def onDetectionsMsg(self, msg):
        self._last_message_ts = datetime.now()
        state_proto = self._robot._state_proto
        state_proto.rplidar.zones.CopyFrom(msg)

    async def onRobotDetectionMsg(self, msg):
        self._last_message_ts = datetime.now()
        state_proto = self._robot._state_proto
        self._detections_ts[msg.id] = datetime.now()
        added = False
        msg.detect_quality = 3
        for d in state_proto.rplidar_detections:
            if d.id == msg.id:
                d.CopyFrom(msg)
                added = True
                break
        if not added:
            state_proto.rplidar_detections.append(msg)
        del state_proto.rplidar.detections[:]
        state_proto.rplidar.detections.extend(state_proto.rplidar_detections)

    async def onEmergencyRaiseMsg(self, msg):
        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!")
        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!")
        LOGGER.debug("!!!!  EMERGENCY  !!!!")
        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!")
        LOGGER.debug("!!!!!!!!!!!!!!!!!!!!!")


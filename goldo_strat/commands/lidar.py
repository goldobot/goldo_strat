import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import struct


class LidarCommands:
    def __init__(self, robot):
        self._robot = robot

    def _publish(self, topic, msg=None):
        return self._robot._broker.publishTopic(topic, msg)

    def start(self):
        return self._publish('rplidar/in/start')

    def stop(self):
        return self._publish('rplidar/in/stop')

    def objectFrontNear(self):
        return self._robot._state_proto.rplidar.zones.front_near

    def objectFrontFar(self):
        return self._robot._state_proto.rplidar.zones.front_far

    def objectBackNear(self):
        return self._robot._state_proto.rplidar.zones.back_near

    def objectBackFar(self):
        return self._robot._state_proto.rplidar.zones.back_far

    def objectInDisk(self, c, r):
        detections = self._robot._state_proto.rplidar.detections
        for d in detections:
            dx = d.x - c[0]
            dy = d.y - c[1]
            if (dx ** 2 + dy ** 2) <= r ** 2:
                return True
        return False

    def objectInRectangle(self, p1, p2):
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]

        if x1 > x2:
            t = x1
            x1 = x2
            x2 = t

        if y1 > y2:
            t = y1
            y1 = y2
            y2 = t
        detections = self._robot._state_proto.rplidar.detections
        for d in detections:
            if x1 <= d.x <= x2 and y1 <= d.y <= y2:
                return True
        return False

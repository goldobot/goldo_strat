import pb2 as _pb2
import google.protobuf as _pb

import asyncio

_sym_db = _pb.symbol_database.Default()


class CameraCommands:
    def __init__(self, robot):
        self._robot = robot
        self._loop = asyncio.get_event_loop()
        self._futures = {}
        self._sequence_number = 0
        self._future_girouette = None

        self._robot._broker.registerCallback('camera/out/detections', self._onMsgDetections)

    async def captureGirouette(self):
        future = self._loop.create_future()
        self._futures[id(future)] = future
        future.add_done_callback(self._remove_future)
        self._future_girouette = future

        await self._robot._broker.publishTopic('camera/in/capture/girouette', None)
        return await future

    async def _onMsgDetections(self, msg):
        if self._future_girouette is not None:
            res = 'unknown'
            for d in msg.detections:
                if d.tag_id == 17:
                    if d.uy > 0.3:
                        res = 'south'
                    if d.uy < -0.3:
                        res = 'north'
            self._future_girouette.set_result(res)
            self._future_girouette = None

    def _remove_future(self, future):
        self._futures.pop(id(future))

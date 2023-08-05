import asyncio
from datetime import datetime
import logging

LOGGER = logging.getLogger(__name__)


class NucleoStateUpdater(object):
    def __init__(self, robot):
        self._robot = robot
        self._broker = robot._broker
        self._nucleo_proto = self._robot._state_proto.nucleo
        self._heartbeat_received = False
        self._ping_trig_cnt = 0
        self._broker.registerCallback('nucleo/out/os/heartbeat', self.onHeartbeatMsg)
        self._broker.registerCallback('nucleo/out/os/reset', self.onResetMessage)
        self._broker.registerCallback('nucleo/out/os/task_statistics/uart_comm', self.onUartCommStatsMsg)
        self._broker.registerCallback('nucleo/out/robot/config/load_status', self.onConfigStatusMsg)
        self._broker.registerCallback('nucleo/out/os/task_statistics/uart_comm', self.onUartCommStatsMsg)
        self._broker.registerCallback('nucleo/out/os/task_statistics/odrive_comm', self.onODriveCommStatsMsg)
        self._broker.registerCallback('nucleo/out/os/task_statistics/propulsion', self.onPropulsionStatsMsg)
        self._broker.registerCallback('nucleo/out/propulsion/odrive/statistics', self.onPropulsionODriveStatsMsg)

        self._last_uart_comm_stats_ts = 0
        self._last_odrive_comm_stats_ts = 0
        self._last_message_ts = datetime.now()
        self._watchdog_task = asyncio.create_task(self.runWatchdog())

    def setBroker(self, broker):
        self._broker = broker

    async def runWatchdog(self):
        while True:
            await asyncio.sleep(1)
            if not self._heartbeat_received:
                self._nucleo_proto.connected = False
            self._heartbeat_received = False

    async def onNucleoReset(self):
        self._nucleo_proto.Clear()
        self._robot.onNucleoReset()

    async def onHeartbeatMsg(self, msg):
        if msg.timestamp + 2000 < self._nucleo_proto.heartbeat:
            LOGGER.warning('NucleoStateUpdater heartbeat error detected: new=%s, old=%s', msg.timestamp, self._nucleo_proto.heartbeat)
            # FIXME : TODO : GOLDO : why disable this?? (don't f**k up the curent sequence after heartbeat error?)
            #await self.onNucleoReset()
        self._nucleo_proto.heartbeat = msg.timestamp
        self._nucleo_proto.connected = True
        self._heartbeat_received = True
        if self._ping_trig_cnt == 9:
            await self._broker.publishTopic('nucleo/in/os/ping', None)
            self._ping_trig_cnt = 0
        else:
            self._ping_trig_cnt = self._ping_trig_cnt + 1

    async def onResetMessage(self, msg):
        # FIXME : TODO : GOLDO : why return here??
        # return
        await self.onNucleoReset()

    async def onConfigStatusMsg(self, msg):
        # config status is inverted in nucleo code
        self._nucleo_proto.configured = not msg.status

    async def onUartCommStatsMsg(self, msg):
        self._last_uart_comm_stats_ts = self._nucleo_proto.heartbeat
        self._nucleo_proto.tasks_statistics.uart_comm.CopyFrom(msg)

    async def onODriveCommStatsMsg(self, msg):
        self._last_odrive_comm_stats_ts = self._nucleo_proto.heartbeat
        self._nucleo_proto.tasks_statistics.odrive_comm.CopyFrom(msg)

    async def onPropulsionODriveStatsMsg(self, msg):
        self._last_odrive_client_stats_ts = self._nucleo_proto.heartbeat
        self._nucleo_proto.odrive.client_statistics.CopyFrom(msg)
        self._nucleo_proto.odrive.synchronized = msg.synchronized

    async def onPropulsionStatsMsg(self, msg):
        self._last_propulsionstats_ts = self._nucleo_proto.heartbeat
        self._nucleo_proto.tasks_statistics.propulsion.CopyFrom(msg)

    async def onFpgaStatsMsg(self, msg):
        pass

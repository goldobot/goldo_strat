import pb2 as _pb2
import google.protobuf as _pb

import asyncio
import math
import os
from pathlib import Path

from .commands import RobotCommands
from .commands import PropulsionCommands
from .commands import ServosCommands
from .commands import LidarCommands
from .commands import ODriveCommands
from .commands import CameraCommands
from .commands import ScopeCommands
from .commands import PneumaticCommands

from .commands import PropulsionError

from .services import SensorsState
from .nucleo.state_updater import NucleoStateUpdater
from .services import RPLidarUpdater
from .enums import *
from .strategy import StrategyEngine
from . import sequences_importer
import importlib

import subprocess
import sys

import logging

import runpy

_sym_db = _pb.symbol_database.Default()
LOGGER = logging.getLogger(__name__)

MATCH_DURATION = 100


class RobotExceptions(object):
    PropulsionError = PropulsionError


_commands = {
    'camera': CameraCommands

}


class RobotMain:
    def sequence(self, func):
        self._sequences[func.__name__] = func
        return func

    def add_option(self, name, default):
        self._options[name] = default

    def loadConfig(self, config_path: Path):
        self._sequences = {}
        config = _pb2.get_symbol('goldo.robot.RobotConfig')()
        if not os.path.exists(config_path / 'robot_config.bin'):
            warn_msg = "WARNING configuration file '{}' missing".format(config_path / 'robot_config.bin')
            LOGGER.debug(warn_msg)
            return
        config.ParseFromString(open(config_path / 'robot_config.bin', 'rb').read())

        LOGGER.debug('RobotMain.loadConfig from %s', config_path)
        self._config_proto = config
        self._sensors_updater.loadConfig()
        self._sequences_globals['servos'].loadConfig()
        self._sequences_globals['robot'].loadConfig()
        # import all sequences
        sequences_importer.meta_finder.unload_all()
        self._options = {}
        sequences_path = config_path / 'sequences'
        sequences_importer.meta_finder.sequences_path = sequences_path
        sequences_importer.meta_finder.inject_globals = self._sequences_globals
        importlib.import_module('sequences.sequences')

    def __init__(self, broker):
        self._broker = broker
        config_path = Path(f'config/test/')
        self._state_proto = _pb2.get_symbol('goldo.robot.RobotState')()
        self._tasks = []
        self._simulation_mode = False
        self.side = 0
        self.start_zone = 0
        self._adversary_detection_enable = True
        self.commands = RobotCommands(self)
        self.propulsion = PropulsionCommands(self)
        self._sensors_updater = SensorsState(self)
        self._state_updater = NucleoStateUpdater(self)
        self._rplidar_updater = RPLidarUpdater(self)
        self.scope = ScopeCommands()
        self._sequences = {}
        self._match_state = MatchState.Idle
        self._match_armed = False
        self._current_task = None
        self._last_girouette = None
        self._futures_propulsion_ack = []
        self._futures_propulsion_wait_stopped = []
        self._futures_match_timer = []

        self.camera = CameraCommands(self)

        self._strategy_engine = StrategyEngine(self)
        self.odrive = ODriveCommands(self)
        self._sequences_globals = {}
        self._sequences_globals['camera'] = self.camera
        self._sequences_globals['scope'] = self.scope
        self._sequences_globals['strategy'] = self._strategy_engine
        self._sequences_globals['odrive'] = self.odrive
        self._sequences_globals['robot'] = self.commands
        self._sequences_globals['propulsion'] = self.propulsion
        self._sequences_globals['sensors'] = self._state_proto.sensors
        self._sequences_globals['servos'] = ServosCommands(self)
        self._sequences_globals['lidar'] = LidarCommands(self)
        self._sequences_globals['pneumatic'] = PneumaticCommands(self)
        self._sequences_globals['rplidar_detections'] = self._state_proto.rplidar_detections
        self._sequences_globals['exceptions'] = RobotExceptions
        self._sequences_globals['logger'] = LOGGER
        self.registerCallbacks()
        self.loadConfig(config_path)
        self._task_match = None
        self._current_task = None
        self._tasks = {}
        self._options = {}
        self._recorder_process = None
        self._task_main_loop = asyncio.create_task(self.runMainLoop())

    async def runMainLoop(self):
        while True:
            await asyncio.sleep(0.1)
            await self._broker.publishTopic('gui/in/robot_state', self._state_proto)
            self._create_task(self.propulsion._on_robot_state(self._state_proto))

            if self._match_state == MatchState.WaitForStartOfMatch and self._state_proto.tirette:
                self._match_armed = True

            if self._match_state == MatchState.WaitForStartOfMatch and self._match_armed and not self._state_proto.tirette:
                LOGGER.info('*******************')
                LOGGER.info('*******************')
                LOGGER.info('**               **')
                LOGGER.info('**  START MATCH  **')
                LOGGER.info('**               **')
                LOGGER.info('*******************')
                LOGGER.info('*******************')
                await self.startMatch()

    async def configNucleo(self, msg=None):
        """Upload the nucleo board configuration."""
        self.onNucleoReset()
        from .nucleo import compile_config

        buff, crc = compile_config(self._config_proto)

        await self._broker.publishTopic('nucleo/in/robot/config/load_begin',
                                        _sym_db.GetSymbol('goldo.nucleo.robot.ConfigLoadBegin')(size=len(buff)))
        # Upload codes by packets
        while len(buff) > 0:
            await self._broker.publishTopic('nucleo/in/robot/config/load_chunk',
                                            _sym_db.GetSymbol('goldo.nucleo.robot.ConfigLoadChunk')(data=buff[0:32]))
            buff = buff[32:]
        # Finish programming
        await self._broker.publishTopic('nucleo/in/robot/config/load_end',
                                        _sym_db.GetSymbol('goldo.nucleo.robot.ConfigLoadEnd')(crc=crc))

        await asyncio.sleep(1)
        await self._rplidar_updater.loadConfig()
        self.propulsion.loadConfig()

        await self._broker.publishTopic('nucleo/in/propulsion/odrive/clear_errors')
        await self._broker.publishTopic('nucleo/in/propulsion/simulation/enable',
                                        _sym_db.GetSymbol('google.protobuf.BoolValue')(value=self._simulation_mode))
        # make sure not callbacks from before reconfig are running
        asyncio.get_event_loop().call_soon(self._broker._cancel_tasks)

    def onNucleoReset(self):
        LOGGER.error("nucleo reset")
        self._match_state = MatchState.Idle
        self._state_proto.match_state = self._match_state
        if self._current_task is not None:
            self._current_task.cancel()

    async def logMessage(self, message, *args):
        await self._broker.publishTopic('main/log/message',
                                        _sym_db.GetSymbol('google.protobuf.StringValue')(value=message.format(*args)))

    async def onConfigStatus(self, msg):
        await self._broker.publishTopic('gui/in/nucleo_config_status', msg)

    async def onDebugStartMatch(self, msg):
        await self.startMatch()

    async def onPreMatch(self, msg):
        if self._current_task is not None:
            self._current_task.cancel()
        await self._broker.publishTopic('match/timer/stop')

        # start a recorder
        if self._recorder_process is not None:
            self._recorder_process.kill()

        # FIXME : DEBUG (recorder disbled by default)
        #self._recorder_process = subprocess.Popen([
        #    sys.executable,
        #    '/home/goldorak/workspace/goldo_strat/messages_recorder.py',
        #    '/home/goldorak/workspace/record_prematch.bin'
        #])

        # return
        LOGGER.info("prematch started, side = {}".format({0: 'unset', 1: 'purple', 2: 'yellow'}[self.side]))
        self._match_state = MatchState.PreMatch
        self._state_proto.match_state = self._match_state

        self._current_task = asyncio.create_task(self._prematchSequence())
        self._current_task.add_done_callback(self.onCurrentTaskDone)

    async def _prematchSequence(self):
        try:
            status = await self._sequences['prematch']()
        except asyncio.CancelledError:
            self._match_state = MatchState.Idle
            self._state_proto.match_state = self._match_state
            LOGGER.error('prematch task cancelled')
            raise
        except Exception as e:
            LOGGER.exception(e)
            self._match_state = MatchState.Idle
            self._state_proto.match_state = self._match_state
            LOGGER.error('prematch failed')
            return

        if status:
            self._match_armed = False
            self._match_state = MatchState.WaitForStartOfMatch
            self._state_proto.match_state = self._match_state
            LOGGER.info('prematch finished')
        else:
            self._match_state = MatchState.Idle
            self._state_proto.match_state = self._match_state
            LOGGER.error('prematch failed')

    async def _matchSequence(self):
        try:
            await asyncio.wait_for(self._strategy_engine.run(), MATCH_DURATION)
        except asyncio.TimeoutError:
            pass
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception('')
        LOGGER.info('match end')
        return

    async def startMatch(self):
        if self._current_task is not None:
            return
        self._match_state = MatchState.Match
        self._state_proto.match_state = self._match_state
        await self._broker.publishTopic('nucleo/in/match/timer/start')
        self.match_timer = 100
        self._state_proto.match_timer = self.match_timer
        await self._broker.publishTopic('gui/in/match_state',
                                        _sym_db.GetSymbol('google.protobuf.Int32Value')(value=self._match_state))
        LOGGER.info('match started')

        self._current_task = asyncio.create_task(self._matchSequence())
        self._current_task.add_done_callback(self.onCurrentTaskDone)

    async def onMatchTimer(self, msg):
        self.match_timer = msg.value
        self._state_proto.match_timer = msg.value
        self._strategy_engine._onMatchTimer(msg.value)

    async def onSequenceExecute(self, name, msg):
        LOGGER.info('RobotMain: execute sequence %s', name)
        self._current_task = asyncio.create_task(self._sequences[name]())
        self._current_task.add_done_callback(self.onCurrentTaskDone)

    def onCurrentTaskDone(self, task):
        try:
            result = task.result()
            LOGGER.debug('task finished: %s', task)
        except Exception:
            LOGGER.exception('')
        self._current_task = None

    def registerCallbacks(self):
        broker = self._broker
        self.scope.setBroker(broker)
        broker.registerCallback('gui/out/side', self.onSetSide)
        broker.registerCallback('gui/out/start_zone', self.onSetStartZone)
        broker.registerCallback('gui/out/commands/config_nucleo', self.configNucleo)
        broker.registerCallback('gui/out/commands/prematch', self.onPreMatch)
        broker.registerCallback('gui/out/commands/debug_start_match', self.onDebugStartMatch)
        broker.registerCallback('nucleo/out/robot/config/load_status', self.onConfigStatus)
        broker.registerCallback('nucleo/out/match/timer', self.onMatchTimer)
        broker.registerCallback('robot/sequence/*/execute', self.onSequenceExecute)
        broker.registerCallback('robot/test_astar', self.onTestAstar)

    async def onSetSide(self, msg):
        self.side = msg.value
        #LOGGER.debug("onSetSide(): side = {}".format({0: 'unset', 1: 'green', 2: 'blue'}[self.side]))

    async def onSetStartZone(self, msg):
        self.start_zone = msg.value
        #LOGGER.debug("onSetStartZone(): zone = " + str(self.start_zone))

    def _create_task(self, aw):
        task = asyncio.create_task(aw)
        self._tasks[id(task)] = task
        task.add_done_callback(self._on_task_done)
        return task

    def _on_task_done(self, task):
        del self._tasks[id(task)]
        try:
            task.result()
        except Exception:
            LOGGER.exception('error in broker callback')

    async def onTestAstar(self, msg):
        LOGGER.info('RobotMain: onTestAstar()')
        await self._strategy_engine.try_astar()

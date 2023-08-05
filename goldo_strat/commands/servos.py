import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import struct
import math
import functools
import logging

LOGGER = logging.getLogger(__name__)


class ServosCommands:
    def __init__(self, robot):
        self._robot = robot
        self._servos_ids = {}
        self._servos_names = []
        self._states_proto = self._robot._state_proto.servos
        self._loop = asyncio.get_event_loop()
        self._futures = {}
        self._futures_moving = {}
        self._futures_by_seq = {}
        self._futures_lift_homing = {}
        self._sequence_number = 0

        self._robot._broker.registerCallback('nucleo/out/servo/ack', self._onMsgAck)
        self._robot._broker.registerCallback('nucleo/out/servo/status/moving', self._onMsgMoving)
        self._robot._broker.registerCallback('nucleo/out/servo/status/states', self._onServoStates)
        self._robot._broker.registerCallback('nucleo/out/lift/homing_done', self._on_msg_homing_done)

    def loadConfig(self):
        self._servos_ids = {}
        self._servos_names = []
        servos_proto = self._robot._config_proto.nucleo.servos
        for i, servo_proto in enumerate(servos_proto):
            self._servos_ids[servo_proto.name] = i
            self._servos_names.append(servo_proto.name)


    async def disableAll(self):
        msg, future = self._create_command_msg('CmdDisableAll')
        await self._robot._broker.publishTopic('nucleo/in/servo/disable_all', msg)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    async def setEnable(self, name_or_servos, enable):
        if isinstance(name_or_servos, (str, bytes)):
            name_or_servos = [name_or_servos]
        ServoEnable = _sym_db.GetSymbol('goldo.nucleo.servos.ServoEnable')
        enables = [ServoEnable(servo_id=self._servos_ids[name], enable=enable) for name in name_or_servos]
        msg, future = self._create_command_msg('CmdSetEnable', enables=enables)
        await self._robot._broker.publishTopic('nucleo/in/servo/enable/set', msg)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    async def setMaxTorque(self, name_or_servos, torque):
        if isinstance(name_or_servos, (str, bytes)):
            name_or_servos = [name_or_servos]
        ServoTorque = _sym_db.GetSymbol('goldo.nucleo.servos.ServoTorque')
        torques = [ServoTorque(servo_id=self._servos_ids[name], torque=math.floor(torque * 255)) for name in
                   name_or_servos]
        msg, future = self._create_command_msg('CmdSetMaxTorques', torques=torques)
        await self._robot._broker.publishTopic('nucleo/in/servo/set_max_torques', msg)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    async def move(self, name, position, speed=1):
        await self.moveMultiple({name: position}, speed)

    async def moveMultiple(self, servos, speed=1):
        speed = int(speed * 0x3ff)
        elts = []
        servos_mask = 0
        seq = self._get_sequence_number()

        for k, v in servos.items():
            id_ = self._servos_ids[k]
            elts.append(_sym_db.GetSymbol('goldo.nucleo.servos.ServoPosition')(servo_id=id_, position=v))
            servos_mask |= (1 << id_)
        msg, future = self._create_command_msg('CmdMoveMultiple', speed=speed, positions=elts)

        await self._robot._broker.publishTopic('nucleo/in/servo/move_multiple', msg)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

        # after the move is started, wait for the servos to stop moving        
        future2 = self._loop.create_future()
        self._futures_moving[id(future2)] = [future2, servos_mask]
        future2.add_done_callback(self._remove_future_moving)
        try:
            await asyncio.wait_for(future2, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    #async def liftDoHoming(self, id_):
    #    future2 = self._loop.create_future()
    #    self._futures_lift_homing[id_] = future2
    #    msg, future = self._create_command_msg('CmdLiftDoHoming')
    #    await self._robot._broker.publishTopic('nucleo/in/lift/do_homing', msg)
    #    await future
    #    await future2
    #    await asyncio.sleep(0.5)

    # FIXME : TODO : activate future2.. (when Nucleo code is ready..)
    async def liftDoHoming(self, id_):
        msg, future = self._create_command_msg('CmdLiftDoHoming', lift_id=id_)
        await self._robot._broker.publishTopic('nucleo/in/lift/do_homing', msg)
        #future2 = self._loop.create_future()
        #self._futures_lift_homing[id(future)] = future2
        #future2.add_done_callback(self._remove_future_homing_done)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)
        #await future2

    async def liftSetEnable(self, id_, enable):
        seq = self._get_sequence_number()
        msg = _sym_db.GetSymbol('goldo.nucleo.servos.CmdLiftSetEnable')(sequence_number=seq, lift_id=id_, enable=enable)
        await self._robot._broker.publishTopic('nucleo/in/lift/set_enable', msg)

    async def liftsRaw(self,target_left=0, speed_left=0, target_right=0, speed_right=0):
        msg, future = self._create_command_msg('CmdLiftsRaw')

        msg.lift1_bltrig = 80
        msg.lift1_speed = speed_left
        msg.lift1_target = target_left

        msg.lift2_bltrig = 80
        msg.lift2_speed = speed_right
        msg.lift2_target = target_right

        await self._robot._broker.publishTopic('nucleo/in/lift/cmd_raw', msg)
        try:
            await asyncio.wait_for(future, 5)
        except asyncio.TimeoutError:
            LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    @property
    def states(self):
        return self._states_proto
        
    async def _onMsgAck(self, msg):
        future = self._futures_by_seq.pop(msg.value, None)
        if future is not None:
            future.set_result(None)

    async def _onMsgMoving(self, msg):
        for e in self._futures_moving.values():
            if not (msg.value & e[1]):
                e[0].set_result(None)

    async def _on_msg_homing_done(self, msg):
        future = self._futures_lift_homing.get(msg.value)
        if future is not None:
            future.set_result(None)

    async def _onServoStates(self, msg):
        for i, s in enumerate(msg.servos):
            self._states_proto[self._servos_names[i]].CopyFrom(s)

    def _get_sequence_number(self):
        seq = self._sequence_number
        self._sequence_number = (self._sequence_number + 1) % 0x0fff
        return seq

    def _remove_future(self, id_, future):
        self._futures.pop(id_, None)

    def _remove_future_moving(self, future):
        self._futures_moving.pop(id(future), None)

    def _remove_future_homing_done(self, future):
        self._futures_lift_homing.pop(id(future), None)

    def _create_future(self):
        future = self._loop.create_future()
        sequence_number = self._get_sequence_number()
        future.add_done_callback(functools.partial(self._remove_future, sequence_number))
        self._futures[sequence_number] = future
        return sequence_number, future

    def _create_command_msg(self, name, **kwargs):
        sequence_number, future = self._create_future()
        self._futures_by_seq[sequence_number] = future
        msg = _sym_db.GetSymbol('goldo.nucleo.servos.' + name)(**kwargs)
        msg.sequence_number = sequence_number
        return msg, future

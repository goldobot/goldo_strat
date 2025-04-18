import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import struct
import math
import functools
import logging

import ctypes
import os

LOGGER = logging.getLogger(__name__)

TYPE_UNKNOWN = 0;
TYPE_STANDARD = 1;
TYPE_DYNAMIXEL_AX12 = 2;
TYPE_DYNAMIXEL_MX28 = 3;
TYPE_GOLDO_LIFT = 4;

LIBDXL_PATH = [".", "/usr/lib"]

LIBDXL_PATH_ENV = os.environ.get("LIBDXL_PATH", None)
if LIBDXL_PATH_ENV:
    LIBDXL_PATH.insert(0, LIBDXL_PATH_ENV)

DEVICE_ID = 0
BAUD_RATE = 1  # Main robot USB2AX

AX_TORQUE_ENABLE_B     = 24
AX_GOAL_POSITION_L     = 30
AX_MOVING_SPEED_L      = 32
AX_MOVING_TORQUE_L     = 34
AX_PRESENT_POSTION_L   = 36
AX_PRESENT_SPEED_L     = 38
AX_PRESENT_LOAD_L      = 40
AX_PRESENT_VOLTAGE     = 42
AX_PRESENT_TEMPERATURE = 43

AX_CW_ANGLE_LIMIT_L    = 6
AX_CCW_ANGLE_LIMIT_L   = 8



class ServosCommands:
    def __init__(self, robot):
        self._robot = robot
        self._servos_local_ids = {}
        self._servos_protos = {}
        self._servos_names = []
        self._states_proto = self._robot._state_proto.servos

        self.hack_seq = 4242

        self.DXL = None
        libdxl = None
        for path in LIBDXL_PATH:
            try:
                libdxl = ctypes.CDLL(path + "/libdxl.so")
            except:
                pass
        if libdxl == None:
            print('Cannot load libdxl.so, check LIBDXL_PATH')
        self.DXL = libdxl
        if self.DXL.dxl_initialize(DEVICE_ID, BAUD_RATE) != 1:
            print('Cannot initialize device USB2AX')
            self.DXL = None

    def loadConfig(self):
        self._servos_local_ids = {}
        self._servos_protos = {}
        self._servos_names = []
        servo_protos = self._robot._config_proto.nucleo.servos
        for i, servo_proto in enumerate(servo_protos):
            self._servos_local_ids[servo_proto.name] = i
            self._servos_protos[servo_proto.name] = servo_proto
            self._servos_names.append(servo_proto.name)
            print ("DEBUG servo : {}".format(i))
            print ("  type        = {}".format(servo_proto.type))
            print ("  id          = {}".format(servo_proto.id))
            print ("  cw_limit    = {}".format(servo_proto.cw_limit))
            print ("  ccw_limit   = {}".format(servo_proto.ccw_limit))
            print ("  max_speed   = {}".format(servo_proto.max_speed))
            print ("  max_torque  = {}".format(servo_proto.max_torque))
            print ("  name        = {}".format(servo_proto.name))


    async def disableAll(self):
        # FIXME : TODO
        pass

    async def setEnable(self, name_or_servos, enable):
        if isinstance(name_or_servos, (str, bytes)):
            name_or_servos = [name_or_servos]

        byte_val = 1 if enable else 0

        for name in name_or_servos:
            servo_proto = self._servos_protos[name]
            LOGGER.debug('setEnable(%s,%d)', name, enable)
            if (servo_proto.type==TYPE_DYNAMIXEL_AX12) or (servo_proto.type==TYPE_DYNAMIXEL_MX28):
                self.DXL.dxl_write_byte(servo_proto.id, AX_TORQUE_ENABLE_B, byte_val)
            elif (servo_proto.type==TYPE_STANDARD):
                # FIXME : DEBUG : HACK
                ServoEnable = _sym_db.GetSymbol('goldo.nucleo.servos.ServoEnable')
                hack_enables = [ServoEnable(servo_id=self._servos_local_ids[name], enable=enable)]
                hack_msg = _sym_db.GetSymbol('goldo.nucleo.servos.CmdSetEnable')(sequence_number=self.hack_seq, enables=hack_enables)
                self.hack_seq = self.hack_seq+1
                await self._robot._broker.publishTopic('nucleo/in/servo/enable/set', hack_msg)
                pass
            else:
                # FIXME : TODO : errmsg?
                pass

        # FIXME : TODO : check command result & synchro
        #try:
        #    await asyncio.wait_for(??future, 5)
        #except asyncio.TimeoutError:
        #    LOGGER.debug('ERROR:timeout on SERVO command %s', msg)

    async def setMaxTorque(self, name_or_servos, torque):
        if isinstance(name_or_servos, (str, bytes)):
            name_or_servos = [name_or_servos]

        uint16_val = math.floor(torque * 0x3ff)

        for name in name_or_servos:
            servo_proto = self._servos_protos[name]
            LOGGER.debug('setMaxTorque(%s,%f)', name, torque)
            if (servo_proto.type==TYPE_DYNAMIXEL_AX12) or (servo_proto.type==TYPE_DYNAMIXEL_MX28):
                self.DXL.dxl_write_word(servo_proto.id, AX_MOVING_TORQUE_L, uint16_val)
            else:
                # FIXME : TODO : errmsg?
                pass

        # FIXME : TODO : check command result & synchro

    async def move(self, name, position, speed=1):
        await self.moveMultiple({name: position}, speed)

    async def moveMultiple(self, servos, speed=1):
        uint16_speed = int(speed * 0x3ff)
        elts = []

        for k, v in servos.items():
            servo_proto = self._servos_protos[k]
            LOGGER.debug('move(name=%s,pos=%d,speed=%f)', k, v, speed)
            if (servo_proto.type==TYPE_DYNAMIXEL_AX12) or (servo_proto.type==TYPE_DYNAMIXEL_MX28):
                self.DXL.dxl_write_word(servo_proto.id, AX_MOVING_SPEED_L, uint16_speed)
                await asyncio.sleep(0.01)
                self.DXL.dxl_write_word(servo_proto.id, AX_GOAL_POSITION_L, v)
                await asyncio.sleep(0.01)
            elif (servo_proto.type==TYPE_STANDARD):
                # FIXME : DEBUG : HACK
                ServoPosition = _sym_db.GetSymbol('goldo.nucleo.servos.ServoPosition')
                hack_poses = [ServoPosition(servo_id=self._servos_local_ids[name], position=v)]
                hack_msg = _sym_db.GetSymbol('goldo.nucleo.servos.CmdMoveMultiple')(sequence_number=self.hack_seq, speed=uint16_speed, positions=hack_poses)
                self.hack_seq = self.hack_seq+1
                await self._robot._broker.publishTopic('nucleo/in/servo/move_multiple', hack_msg)
                pass
            else:
                # FIXME : TODO : errmsg?
                pass

        # FIXME : TODO : check command result & synchro


    @property
    def states(self):
        # FIXME : TODO
        return self._states_proto
        

# FIXME : DEBUG : HACK

    async def liftDoHoming(self, id_):
        hack_msg = _sym_db.GetSymbol('goldo.nucleo.servos.CmdLiftDoHoming')(sequence_number=self.hack_seq, lift_id=id_)
        self.hack_seq = self.hack_seq+1
        await self._robot._broker.publishTopic('nucleo/in/lift/do_homing', hack_msg)

    async def liftSetEnable(self, id_, enable):
        hack_msg = _sym_db.GetSymbol('goldo.nucleo.servos.CmdLiftSetEnable')(sequence_number=self.hack_seq, lift_id=id_, enable=enable)
        self.hack_seq = self.hack_seq+1
        await self._robot._broker.publishTopic('nucleo/in/lift/set_enable', hack_msg)

    async def liftsRaw(self,target_left=0, speed_left=0, target_right=0, speed_right=0):
        hack_msg = self._create_command_msg('CmdLiftsRaw')
        _sym_db.GetSymbol('goldo.nucleo.servos.CmdLiftsRaw')(sequence_number=self.hack_seq)
        self.hack_seq = self.hack_seq+1
        msg.lift1_bltrig = 80
        msg.lift1_speed = speed_left
        msg.lift1_target = target_left
        msg.lift2_bltrig = 80
        msg.lift2_speed = speed_right
        msg.lift2_target = target_right
        await self._robot._broker.publishTopic('nucleo/in/lift/cmd_raw', hack_msg)
        # FIXME : DEBUG : awfull hack!
        await asyncio.sleep(3.0)

    async def autotest(self, name, position):
        print ("AUTOTEST:")
        print ("  name    ={}".format(name))
        print ("  position={}".format(position))
        servo_proto = self._servos_protos[name]
        print ("  id={}".format(servo_proto.id))
        await asyncio.sleep(2.0)


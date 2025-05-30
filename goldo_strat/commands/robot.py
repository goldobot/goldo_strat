import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()
import asyncio
import struct
import math

import runpy


class RobotCommands:
    def __init__(self, robot):
        self._robot = robot
        self._broker = self._robot._broker
        self._apb_addr = 0
        self._apb_data = 0
        self._broker.registerCallback('nucleo/out/fpga/reg', self.onFpgaRegRead)
        
    def loadConfig(self):
        self._gpio_ids = {}
        for gpio_proto in self._robot._config_proto.nucleo.hal.gpio:
            name = gpio_proto.name
            self._gpio_ids[name] = gpio_proto.id

    def _publish(self, topic, msg=None):
        return self._robot._broker.publishTopic(topic, msg)

    def sequence(self, func):
        return self._robot.sequence(func)

    @property
    def score(self):
        return self._robot._state_proto.score

    async def setScore(self, score):
        self._robot._state_proto.score = score

        await self._publish('gui/in/score',
                            _sym_db.GetSymbol('google.protobuf.Int32Value')(value=self._robot._state_proto.score))

    @property
    def side(self):
        return self._robot.side
    
    @property
    def start_zone(self):
        return self._robot.start_zone

    @property
    def sensors(self):
        return self._robot._state_proto.sensors
        
    async def gpioSet(self, name, value):
        gpio_id = self._gpio_ids[name]
        await self._publish('nucleo/in/gpio/set',
                            _sym_db.GetSymbol('goldo.nucleo.gpio.CmdGpioSet')(sequence_number=0, gpio_id=gpio_id, value=value))

    async def fpgaRegWrite(self, reg_addr, value):
        #print ("  fpgaRegWrite() : addr={:8x} data={:8x}".format(reg_addr, value))
        await self._publish('nucleo/in/fpga/reg/write',
                            _sym_db.GetSymbol('goldo.nucleo.fpga.RegWrite')(apb_address = reg_addr, apb_value = value))

    async def fpgaRegRead(self, reg_addr):
        self._apb_addr = 0
        self._apb_data = -1
        await self._publish('nucleo/in/fpga/reg/read',
                            _sym_db.GetSymbol('goldo.nucleo.fpga.RegRead')(apb_address = reg_addr))
        for i in range(0,10):
            await asyncio.sleep(0.02)
            if (self._apb_addr == reg_addr):
                break
        #print ("  fpgaRegRead() : addr={:8x} data={:8x}".format(self._apb_addr, self._apb_data))
        return self._apb_data

    async def onFpgaRegRead(self, msg):
        self._apb_addr = msg.apb_address
        self._apb_data = msg.apb_value
        #print ("  onFpgaRegRead() : addr={:8x} data={:8x}".format(self._apb_addr, self._apb_data))



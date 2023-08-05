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
        
    @property
    def tryOhm(self):
        k = (self.sensors['tryohm_bit_0'], self.sensors['tryohm_bit_1'])
        m = {
            (True, True): None,
            (True, False): 'yellow',
            (False, True): 'red',
            (False, False): 'purple'
            }
        return m.get(k)
        
    async def gpioSet(self, name, value):
        gpio_id = self._gpio_ids[name]
        await self._publish('nucleo/in/gpio/set',
                            _sym_db.GetSymbol('goldo.nucleo.gpio.CmdGpioSet')(sequence_number=0, gpio_id=gpio_id, value=value))

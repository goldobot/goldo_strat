import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()

msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.ExecuteFaceDirection')()


class SensorsState:
    def __init__(self, robot):
        self._robot = robot
        self._sensors_map = {}
        self._robot._broker.registerCallback('nucleo/out/sensors/state', self.onSensorsStateMsg)

    def loadConfig(self):
        self._sensors_proto = self._robot._state_proto.sensors
        self._sensors_proto.clear()
        for i, sensor_proto in enumerate(self._robot._config_proto.nucleo.sensors):
            name = sensor_proto.name
            self._sensors_map[i] = name
            self._sensors_proto[name] = False

    async def onSensorsStateMsg(self, msg):
        state = msg.state
        for k, v in self._sensors_map.items():
            self._sensors_proto[v] = (state & (1 << k)) != 0
        self._robot._state_proto.tirette = not self._sensors_proto.get('tirette', True)
        self._robot._state_proto.emergency_stop = self._sensors_proto.get('emergency_stop', True)

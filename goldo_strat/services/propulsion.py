import google.protobuf as _pb
import logging

LOGGER = logging.getLogger(__name__)

_sym_db = _pb.symbol_database.Default()

msg = _sym_db.GetSymbol('goldo.nucleo.propulsion.ExecuteFaceDirection')()


class PropulsionService:
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

    async def _onTelemetryMsg(self, msg):
        self._robot._state_proto.robot_pose.CopyFrom(msg.pose)
        self.state = msg.state

    async def _on_controller_event(self, msg):
        # reposition
        if msg.type == 1:
            self._reposition_event = msg

    async def _on_cmd_event(self, msg):
        if msg.status == 4:
            LOGGER.debug('propulsion cmd ack', msg.sequence_number)
            future = self._futures_ack.get(msg.sequence_number)
            if future is not None:
                future.set_result(None)

        future = self._futures.get(msg.sequence_number)

        if future is not None:
            if msg.status == 1:
                future.set_result(None)
                return
            if msg.status == 2:
                future.set_exception(PropulsionError(msg.error))
                return
            if msg.status == 3:
                future.set_exception(PropulsionError(msg.error))
                return
            if msg.status == 4 and not self._cmd.get(msg.sequence_number):
                future.set_result(None)
                return

        

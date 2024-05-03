from goldo_strat.zmq_broker_interface import ZmqBrokerInterface
import asyncio
import setproctitle
import logging
import datetime


from pathlib import Path

import sys
import signal

from optparse import OptionParser

def rm_tree(path: Path):
    for child in path.iterdir():
        if child.is_file():
            child.unlink()
        else:
            rm_tree(child)
    path.rmdir()


def goldo_signal_handler(my_broker,sig,frame):
    print (" DEBUG GOLDO : sys.exit(0)")
    #my_broker._process.terminate()
    sys.exit(0)

async def config_put(config_name, msg):
    config_path = Path(f'config/{config_name}')
    config_path.mkdir(parents=True, exist_ok=True)
    (config_path / 'sequences').mkdir(parents=True, exist_ok=True)
    open(config_path / 'robot_config.bin', 'wb').write(msg.SerializeToString())
    for sequences_file in msg.sequences_files:
        open(config_path / 'sequences' / sequences_file.path, 'w').write(sequences_file.body)
    robot.loadConfig(config_path)
    
async def config_delete(config_name, msg):
    config_path = Path(f'config/{config_name}')
    rm_tree(config_path)
    
async def config_set_default(config_name, msg):
    config_path = Path('config/')
    config_path.mkdir(exist_ok=True)
    open(config_path / 'default', 'w').write(config_name)
    
async def main():
    from goldo_strat.robot_main import RobotMain
    from goldo_strat.log_handler import GoldoLogHandler
    global robot

    parser = OptionParser()
    parser.add_option('--mock', default=False)

    #(options, args) = parser.parse_args(sys.argv)

    
    broker = ZmqBrokerInterface()
    handler = GoldoLogHandler(broker)
    
    logger = logging.getLogger('goldo_strat')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    
    robot = RobotMain(broker)
    if 'simulation' in sys.argv:
        robot._simulation_mode = True
    if 'interactive' in sys.argv:
        signal.signal(signal.SIGINT, lambda sig,frame: goldo_signal_handler(broker,sig,frame))
    broker.registerCallback('config/*/put', config_put)
    broker.registerCallback('config/*/delete', config_delete)
    broker.registerCallback('config/*/set_default', config_set_default)
    broker.registerCallback('robot/config_nucleo', robot.configNucleo)
    broker.registerCallback('camera/out/image', lambda msg: broker.publishTopic('gui/in/camera/image', msg))
    broker.registerCallback('camera/out/detections', lambda msg: broker.publishTopic('gui/in/camera/detections', msg))
    broker.registerCallback('nucleo/out/propulsion/telemetry', lambda msg: broker.publishTopic('rplidar/in/robot_pose', msg.pose)) 
    broker.registerCallback('nucleo/out/match/timer', lambda msg: broker.publishTopic('gui/in/match_timer', msg))         
    broker.registerForward('nucleo/out/os/heartbeat', 'gui/in/heartbeat')
    #broker.startRecording()
    await broker.run()

if __name__ == '__main__':


    
    now=datetime.datetime.now()
    
    print("START goldo_strat {:s}".format(now.strftime("%Y-%m-%d %H:%M:%S")))
    setproctitle.setproctitle('goldo_strat')
    
    
    logger = logging.getLogger('goldo_strat.robot_main')
    #logger.setLevel(logging.INFO)
    logger.setLevel(logging.DEBUG)
    
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)

    logger.addHandler(consoleHandler)

    debug_modules = [
        'goldo_strat.commands.propulsion',
        'goldo_strat.services.rplidar',
        'goldo_strat.zmq_broker_interface'
    ]

    for m in debug_modules:
        logger = logging.getLogger(m)
        logger.setLevel(logging.DEBUG)
        logger.addHandler(consoleHandler)

    logger = logging.getLogger('goldo_strat.strategy.strategy_engine_base')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    
    logger = logging.getLogger('goldo_strat.strategy.strategy_engine')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)

    logger = logging.getLogger('goldo_strat.nucleo.state_updater')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

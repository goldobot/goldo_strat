import asyncio
import zmq
import zmq.utils.monitor
from zmq.asyncio import Context, Poller
import struct
import enum
import re
import os
import socket
import logging

import multiprocessing
from multiprocessing import Process, Pipe

import pb2 as _pb2
import google.protobuf as _pb

_sym_db = _pb.symbol_database.Default()


LOGGER = logging.getLogger(__name__)

# FIXME : TODO : import package for broker definitions..
class ZmqBrokerCmd(enum.Enum):
    PUBLISH_TOPIC = 0
    REGISTER_CALLBACK = 1
    REGISTER_FORWARD = 2


class ZmqBrokerInterface():
    def __init__(self):
        #ctx = multiprocessing.get_context('spawn')
        #self._conn, child_conn = ctx.Pipe()
        #self._process = ctx.Process(target=run_broker_process, args=(child_conn,))
        #self._process.start()

        local_ip = "127.0.0.1"

        self._zmq_context = Context.instance()

        self._poller = Poller()

        self._socket_in = self._zmq_context.socket(zmq.SUB)
        self._socket_in.connect('tcp://{}:3701'.format(local_ip))
        self._socket_in.setsockopt(zmq.SUBSCRIBE,b'')

        self._socket_out = self._zmq_context.socket(zmq.PUB)
        self._socket_out.connect('tcp://{}:3702'.format(local_ip))

        self._message_available = asyncio.Event()
        self._message_available.clear()

        #loop = asyncio.get_event_loop()
        #loop.add_reader(self._socket_in.fileno(), self._message_available.set)
        self._poller.register(self._socket_in, zmq.POLLIN)

        self._tasks = {}

        self._callbacks = []

    def registerCallback(self, pattern: str, callback):
        pattern = (
            pattern
                .replace('*', r'([^/]+)')
                .replace('/#', r'/([^/]+)*')
                .replace('#/', r'([^/]+)/*')
        )
        cmd_code = ZmqBrokerCmd.REGISTER_CALLBACK.value
        new_cb_id = len(self._callbacks)
        self._socket_out.send_multipart([cmd_code.to_bytes(1,'little'), pattern.encode('utf8'), new_cb_id.to_bytes(4,'little')])
        self._callbacks.append((re.compile(f"^{pattern}$"), new_cb_id, callback))

    def registerForward(self, pattern: str, forward_str: str):
        pattern = (
            pattern
                .replace('*', r'([^/]+)')
                .replace('/#', r'/([^/]+)*')
                .replace('#/', r'([^/]+)/*')
        )
        cmd_code = ZmqBrokerCmd.REGISTER_FORWARD.value
        self._socket_out.send_multipart([cmd_code.to_bytes(1,'little'), pattern.encode('utf8'), forward_str.encode('utf8')])

    async def run(self):
        while True:
            #await self._message_available.wait()
            #self._message_available.clear()
            events = await self._poller.poll()

            for s, e in events:
                if e & zmq.POLLIN:
                    flags = s.getsockopt(zmq.EVENTS)
                    while flags & zmq.POLLIN:
                        if s == self._socket_in:
                            topic_b, msg_class_name_b, msg_b = await s.recv_multipart()
                            topic = topic_b.decode('utf8')
                            msg_class_name = msg_class_name_b.decode('utf8')
                            msg_class = _sym_db.GetSymbol(msg_class_name)
                            if msg_class is not None:
                                msg = msg_class()
                                msg.ParseFromString(msg_b)
                            else:
                                msg = _sym_db.GetSymbol('google.protobuf.Empty')()
                            await self.onTopicReceived(topic, msg, 0)

                            #print(topic, msg_class_name)

    async def onTopicReceived(self, topic, msg, dummy):
        if msg is None:
            msg = _sym_db.GetSymbol('google.protobuf.Empty')()

        callback_matches = tuple((regexp.match(topic), callback_id, callback_func) for regexp, callback_id, callback_func in self._callbacks)
        callbacks_list = tuple((callback_func, tuple(match.groups())) for match, callback_id, callback_func in callback_matches if match)

        for callback_func, groups in callbacks_list:
            self._create_task(callback_func(*groups, msg))

    async def publishTopic(self, topic, msg=None):
        if msg is None:
            msg = _sym_db.GetSymbol('google.protobuf.Empty')()
        cmd_code = ZmqBrokerCmd.PUBLISH_TOPIC.value
        self._socket_out.send_multipart([cmd_code.to_bytes(1,'little'), topic.encode('utf8'), msg.DESCRIPTOR.full_name.encode('utf8'), msg.SerializeToString()])

    def _cancel_tasks(self):
        for t in self._tasks.values():
            t.cancel()

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

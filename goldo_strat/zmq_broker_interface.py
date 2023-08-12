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

# FIXME : TODO : REMOVE (reminder of the old broker implementation..)
class ZmqBrokerCmd(enum.Enum):
    PUBLISH_TOPIC = 0
    REGISTER_CALLBACK = 1
    REGISTER_FORWARD = 2


class ZmqBrokerInterface():
    def __init__(self):
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
        new_cb_id = len(self._callbacks)
        self._callbacks.append((re.compile(f"^{pattern}$"), new_cb_id, callback))
        cmd_topic = "broker/admin/cmd/register_callback"
        cmd_msg = _sym_db.GetSymbol('google.protobuf.StringValue')(value = pattern)
        self._create_task(self.publishTopic(cmd_topic, cmd_msg))

    def registerForward(self, pattern: str, forward_str: str):
        pattern = (
            pattern
                .replace('*', r'([^/]+)')
                .replace('/#', r'/([^/]+)*')
                .replace('#/', r'([^/]+)/*')
        )
        cmd_topic = "broker/admin/cmd/register_forward"
        cmd_msg = _sym_db.GetSymbol('google.protobuf.StringValue')(value = pattern+">"+forward_str)
        self._create_task(self.publishTopic(cmd_topic, cmd_msg))

    async def run(self):
        while True:
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
                            await self.onTopicReceived(topic, msg)

    async def onTopicReceived(self, topic, msg):
        if msg is None:
            msg = _sym_db.GetSymbol('google.protobuf.Empty')()

        callback_matches = tuple((regexp.match(topic), callback_id, callback_func) for regexp, callback_id, callback_func in self._callbacks)
        callbacks_list = tuple((callback_func, tuple(match.groups())) for match, callback_id, callback_func in callback_matches if match)

        for callback_func, groups in callbacks_list:
            self._create_task(callback_func(*groups, msg))

    async def publishTopic(self, topic, msg=None):
        if msg is None:
            msg = _sym_db.GetSymbol('google.protobuf.Empty')()
        self._socket_out.send_multipart([topic.encode('utf8'), msg.DESCRIPTOR.full_name.encode('utf8'), msg.SerializeToString()])

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

import sys
import struct
import socket
import subprocess

import logging
import pb2 as _pb2

import asyncio
import queue

LogMessage = _pb2.get_symbol('goldo.log.LogMessage')

class GoldoLogHandler(logging.Handler):
    def __init__(self, zmq_client):
        logging.Handler.__init__(self)
        self.zmq_client = zmq_client
        self.loop = asyncio.get_event_loop()
        self.queue = queue.Queue()
        self._task = asyncio.create_task(self.run())

        self.mcast_intf = self._read_ip()

        if self.mcast_intf != None:
            l = self.mcast_intf.split('.')
            self.mcast_addr = "231.10.66." + l[3]
            #print("mcast_addr = {}".format(self.mcast_addr))
            self.mcast_port = int(4242)
            self.snd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.snd.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,17)
            self.snd.bind((self.mcast_intf,self.mcast_port))
            mreq=struct.pack("4s4s",socket.inet_aton(self.mcast_addr),socket.inet_aton(self.mcast_intf))
            self.snd.setsockopt(socket.IPPROTO_IP,socket.IP_ADD_MEMBERSHIP,mreq)

    async def run(self):
        while True:
            await asyncio.sleep(0.1)
            try:
                while True:
                    if self.mcast_intf == None:
                        msg = self.queue.get(False)  # non blocking
                        await self.zmq_client.publishTopic('goldo_strat/log', msg)
                    else:
                        msg = self.queue.get(False)  # non blocking
                        my_text = msg.message
                        pkt_buf = bytearray()
                        pkt_buf.extend(map(ord,my_text))
                        self.snd.sendto(pkt_buf, (self.mcast_addr,self.mcast_port))
            except queue.Empty:
                pass

    def prepare(self, record):
        msg = self.format(record)
        proto = LogMessage()
        proto.name = record.name
        proto.message = msg
        proto.pathname = record.pathname
        proto.lineno = record.lineno
        proto.levelno = record.levelno
        proto.func = record.funcName

        # record.args = None
        # record.exc_info = None
        # record.exc_text = None
        return proto

    def emit(self, record):
        try:
            self.queue.put_nowait(self.prepare(record))
        except Exception:
            self.handleError(record)

    def _read_ip(self):
        try:
            self._ip_address = ""
            result_b = subprocess.check_output(["ip", "a"])
            result_s = result_b.decode("utf-8")
            for li in result_s.split('\n'):
                tok = li.split()
                if len(tok) > 0 and tok[0] == "inet":
                    for t in tok:
                        if t.startswith("wlan"):
                            _ip_address = tok[1].split('/')[0]
                            return _ip_address
        except:
            pass
        return None

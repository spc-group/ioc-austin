#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
import re
import enum
import struct
import socket

from caproto import ChannelType, SkipWrite
from caproto.server import (
    PVGroup,
    pvproperty,
    PvpropertyDouble,
    PvpropertyShort,
    PvpropertyShortRO,
    PvpropertyChar,
    SubGroup,
    scan_wrapper,
)


HEARTBEAT_PERIOD = 15


def heartbeat_message(magic_number: int, incarnation: int|float,
                      current_time: int|float, heartbeat_value: int, period: int|float,
                      flags: int, return_port: int, user_message: int, ioc_name: str):
    """Create a UDP message that tells the server the IOC is alive.

    Convert the various parameters into a UDP message based on the
    EPICS alive record specification:

    https://epics-modules.github.io/alive/aliveRecord.html

    """
    EPICS_TIME_CORRECTION = -631152000
    PROTOCOL_VERSION = 5
    msg = bytes()
    msg += struct.pack(">L", magic_number)  # 0-3
    msg += struct.pack(">H", PROTOCOL_VERSION)  # 4-5
    msg += struct.pack(">L", round(incarnation) + EPICS_TIME_CORRECTION)  # 6-9
    msg += struct.pack(">L", round(current_time) + EPICS_TIME_CORRECTION)  # 10-13
    msg += struct.pack(">L", heartbeat_value)  # 14-17
    msg += struct.pack(">H", period)  # 18-19
    msg += struct.pack(">H", flags)  # 20-21
    msg += struct.pack(">H", return_port)  # 22-23
    msg += struct.pack(">L", user_message)  # 24-*
    msg += ioc_name.encode('ascii')
    # Null terminator
    msg += b'\x00'
    return msg


def epics_time(unix_time):
    return unix_time + 631152000


class AliveGroup(PVGroup):
    incarnation: int

    class HostReadStatus(enum.IntEnum):
        IDLE = 0
        QUEUED = 1
        DUE = 2
        OVERDUE = 3

    class InformationPortStatus(enum.IntEnum):
        UNDETERMINED = 0
        OPERABLE = 1
        INOPERABLE = 2

    def __init__(self, sock: socket.socket=None, remote_host: str="localhost", remote_port: int=5678, *args, **kwargs):
        if sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM | socket.SOCK_NONBLOCK)
        self.sock = sock
        super().__init__(*args, **kwargs)
        self.default_remote_host = remote_host
        self.default_remote_port = remote_port

    async def resolve_hostname(self, hostname):
        """Determine the host's IP address.

        Returns
        =======
        addr
          The resolved hostname.
        """
        loop = self.async_lib.get_running_loop()
        addr = await loop.run_in_executor(
            None, socket.gethostbyname, hostname,
        )
        return addr

    def server_address(self):
        """Determine which server to use for sending heartbeats.

        Checks the .RADDR and .AADDR fields to determine which to use.

        Returns
        =======
        addr
          The IP address of the host.
        port
          The port for the heartbeat message.

        """
        invalid_addrs = ["", "invalid AHOST", "invalid RHOST"]
        addr = None
        port = None
        # Check for a valid remote host
        addr_is_valid = self.raddr.value not in invalid_addrs
        port_is_valid = self.rport.value > 0
        if addr_is_valid and port_is_valid:
            addr = self.raddr.value
            port = self.rport.value
        # Check for a valid auxillary host
        addr_is_valid = self.aaddr.value not in invalid_addrs
        port_is_valid = self.aport.value > 0
        if addr_is_valid and port_is_valid:
            addr = self.aaddr.value
            port = self.aport.value
        return (addr, port)

    async def send_heartbeat(self) -> bool:
        """Send a heartbeat message to the alive server.

        Returns
        =======
        heartbeat_sent
          True if the heartbeat message was sent, otherwise false.
        
        """
        if self.hrtbt.value is False:
            return False
        # Build TCP trigger flags
        flags = 0
        if bool(self.itrig.value):
            flags |= 0b1
        if bool(self.isup.value):
            flags |= 0b10
        # Prepare the UDP message
        next_heartbeat = self.val.value + 1
        message = heartbeat_message(
            magic_number=self.hmag.value,
            incarnation=self.incarnation,
            current_time=epics_time(time.time()),
            heartbeat_value=next_heartbeat,
            period=self.hprd.value,
            flags=flags,
            return_port=self.iport.value,
            user_message=self.msg.value,
            ioc_name="",
        )
        # Send the message
        loop = self.async_lib.get_running_loop()
        addr, port = self.server_address()
        if addr is not None and port is not None:
            print(f"Sending UDP to {(addr, port)}, #{next_heartbeat}")
            await loop.sock_sendto(sock=self.sock, data=message, address=(addr, port))
            await self.val.write(next_heartbeat)
        else:
            print("Skipping heartbeat")
    
    val = pvproperty(
        name=".VAL",
        value=0,
        dtype=int,
        doc="Heartbeat Value",
        read_only=True
    )

    @val.scan(HEARTBEAT_PERIOD)
    async def val(self, instance, async_lib):
        # Send heartbeat message
        await self.send_heartbeat()

    @val.startup
    async def val(self, instance, async_lib):
        self.async_lib = async_lib.library
        self.incarnation = epics_time(time.time())
        await self.rhost.write(self.default_remote_host)
        await self.rport.write(self.default_remote_port)

    rhost = pvproperty(
        name=".RHOST",
        value="",
        dtype=ChannelType.STRING,
        doc="Remote Host Name or IP Address",
        read_only=True,
    )
    @rhost.putter
    async def rhost(self, instance, value):
        addr = await self.resolve_hostname(value)
        await self.raddr.write(addr)
    
    raddr = pvproperty(
        name=".RADDR",
        value="",
        dtype=ChannelType.STRING,
        doc="Remote Host IP Address",
        read_only=True,
    )
    rport = pvproperty(
        name=".RPORT",
        value=0,
        dtype=int,
        doc="Remote Host UDP Port Number",
        read_only=True,
    )
    rrsts = pvproperty(
        name=".RRSTS",
        value=HostReadStatus.IDLE,
        dtype=HostReadStatus,
        doc="Remote Host Read Status",
        read_only=True,
    )
    ahost = pvproperty(
        name=".AHOST",
        value="",
        dtype=ChannelType.STRING,
        doc="Aux. Remote Host Name or IP Address",
        read_only=False,
    )
    @ahost.putter
    async def ahost(self, instance, value):
        addr = await self.resolve_hostname(value)
        await self.aaddr.write(addr)

    aaddr = pvproperty(
        name=".AADDR",
        value="",
        dtype=ChannelType.STRING,
        doc="Aux. Remote Host IP Address",
        read_only=True,
    )
    aport = pvproperty(
        name=".APORT",
        value=0,
        dtype=int,
        doc="Aux. Remote Host UDP Port Number",
        read_only=False,
    )
    arsts = pvproperty(
        name=".ARSTS",
        value=HostReadStatus.IDLE,
        dtype=HostReadStatus,
        doc="Aux. Remote Host Read Status",
        read_only=True,
    )
    hrtbt = pvproperty(
        name=".HRTBT", value=False, dtype=bool, doc="Heartbeating State", read_only=False
    )
    hprd = pvproperty(
        name=".HPRD", value=HEARTBEAT_PERIOD, dtype=int, doc="Heartbeat Period", read_only=True
    )
        
    iocnm = pvproperty(
        name=".IOCNM",
        value="",
        dtype=ChannelType.STRING,
        doc="IOC Name Value",
        read_only=True,
    )
    hmag = pvproperty(
        name=".HMAG",
        value=305419896,
        dtype=int,
        doc="Heartbeat Magic Number",
        read_only=True,
    )
    msg = pvproperty(
        name=".MSG", value=0, dtype=int, doc="Message to Send", read_only=False
    )
    iport = pvproperty(
        name=".IPORT",
        value=0,
        dtype=int,
        doc="TCP Information Port Number",
        read_only=True,
    )
    ipsts = pvproperty(
        name=".IPSTS",
        value=InformationPortStatus.UNDETERMINED,
        dtype=InformationPortStatus,
        doc="Information Port Status",
        read_only=True,
    )
    itrig = pvproperty(
        name=".ITRIG",
        value=False,
        dtype=bool,
        doc="Trigger Information Request",
        read_only=False,
    )
    isup = pvproperty(
        name=".ISUP",
        value=False,
        dtype=bool,
        doc="Suppress Information Requests",
        read_only=False,
    )
    ver = pvproperty(
        name=".VER",
        value="5.1.3",
        dtype=ChannelType.STRING,
        doc="Record Version",
        read_only=True,
    )
    evd1 = pvproperty(
        name=".EVD1",
        value="",
        dtype=ChannelType.STRING,
        doc="Default Environment Variable Name 1",
        read_only=True,
    )
    evd16 = pvproperty(
        name=".EVD16",
        value="",
        dtype=ChannelType.STRING,
        doc="Default Environment Variable Name 16",
        read_only=True,
    )
    ev1 = pvproperty(
        name=".EV1",
        value="",
        dtype=ChannelType.STRING,
        doc="Environment Variable Name 1",
        read_only=False,
    )
    ev16 = pvproperty(
        name=".EV16",
        value="",
        dtype=ChannelType.STRING,
        doc="Environment Variable Name 16",
        read_only=False,
    )
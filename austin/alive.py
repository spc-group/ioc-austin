#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
import re
import enum

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


class AliveGroup(PVGroup):

    class HostReadStatus(enum.IntEnum):
        IDLE = 0
        QUEUED = 1
        DUE = 2
        OVERDUE = 3

    class InformationPortStatus(enum.IntEnum):
        UNDETERMINED = 0
        OPERABLE = 1
        INOPERABLE = 2
    
    val = pvproperty(
        name=".VAL",
        value=0,
        dtype=int,
        doc="Heartbeat Value",
        read_only=True
    )
    rhost = pvproperty(
        name=".RHOST",
        value="",
        dtype=ChannelType.STRING,
        doc="Remote Host Name or IP Address",
        read_only=True,
    )
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
        name=".HPRD", value=15, dtype=int, doc="Heartbeat Period", read_only=True
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
        value="",
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

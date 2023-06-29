#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
from typing import Literal
import enum

from caproto import ChannelType
from caproto.server import (
    PVGroup,
    ioc_arg_parser,
    pvproperty,
    run,
    PvpropertyDouble,
    PvpropertyShort,
    PvpropertyShortRO,
    PvpropertyChar,
    SubGroup,
    pvfunction,
)

from .driver import RobotDriver

log = logging.getLogger(__name__)


class ActionsGroup(PVGroup):
    class DanceStyles(enum.IntEnum):
        JAZZ = 0
        BREAK = 1
        TAP = 2

    @pvfunction(default=[0], prefix="dance:")
    async def dance(self, style: ChannelType.STRING = ["jazz"]) -> bool:
        print(f"Dancing with style: '{style}'")
        await asyncio.sleep(5)
        print("done dancing")
        return True

    @pvfunction(default=[0], prefix="pick:")
    async def pick(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        print(f"Running ``pick()`` with {i=}, {j=}, {k=}, {l=}, {m=}, {n=}")

    @pvfunction(default=[0], prefix="place:")
    async def place(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        print(f"Running ``place()`` with {i=}, {j=}, {k=}, {l=}, {m=}, {n=}")

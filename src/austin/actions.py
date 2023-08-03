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
    """PVs for RPC actions that the robot can perform."""

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
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        print(self.parent)
        self.parent.parent.driver.pickj([i, j, k, l, m, n], acc, vel)
        print(f"Running ``pick()`` at {i=}, {j=}, {k=}, {l=}, {m=}, {n=}")

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
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        self.parent.parent.driver.placej([i, j, k, l, m, n], acc, vel)
        print(f"Running ``place()`` at {i=}, {j=}, {k=}, {l=}, {m=}, {n=}")

    @pvfunction(default=[0], prefix="home:")
    async def home(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        self.parent.parent.driver.movej([i, j, k, l, m, n], acc, vel)
        print(f"Running ``home()`` to {i=}, {j=}, {k=}, {l=}, {m=}, {n=}")

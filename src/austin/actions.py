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

    @pvfunction(default=[0], prefix="pickj:")
    async def pickj(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        """Instruct the robot to pick up the sample given in joint positions."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        # Execute the pick function
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None,
            self.parent.parent.driver.pickj,
            [i, j, k, l, m, n],
            acc,
            vel,
            120,
            200,
            120,
            50,
        )

    @pvfunction(default=[0], prefix="pickl:")
    async def pickl(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
    ) -> int:
        """Instruct the robot to pick up the sample given in Cartesian coordinates."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        # Execute the pick function
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None,
            self.parent.parent.driver.pickl,
            [x, y, z, rx, ry, rz],
            acc,
            vel,
            120,
            200,
            120,
            50,
        )

    @pvfunction(default=[0], prefix="placej:")
    async def placej(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        """Instruct the robot to place its sample at the location given in joint positions."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None,
            self.parent.parent.driver.placej,
            [i, j, k, l, m, n],
            acc,
            vel,
            120,
            200,
            120,
            50,
        )

    @pvfunction(default=[0], prefix="placel:")
    async def placel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
    ) -> int:
        """Instruct the robot to place its sample at the location given in Cartesian coordinates."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None,
            self.parent.parent.driver.placel,
            [x, y, z, rx, ry, rz],
            acc,
            vel,
            120,
            200,
            120,
            50,
        )
        print(f"Running ``place()`` at {x=}, {y=}, {z=}, {rx=}, {ry=}, {rz=}")

    @pvfunction(default=[0], prefix="homej:")
    async def homej(
        self,
        i: float = 0.0,
        j: float = 0.0,
        k: float = 0.0,
        l: float = 0.0,
        m: float = 0.0,
        n: float = 0.0,
    ) -> int:
        """Instruct the robot to return to a home position given in joint positions."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None, self.parent.parent.driver.movej, [i, j, k, l, m, n], acc, vel
        )

    @pvfunction(default=[0], prefix="homel:")
    async def homel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
    ) -> int:
        """Instruct the robot to return to a home position given in Cartesian coordinates."""
        acc = self.parent.parent.status.acceleration.value
        vel = self.parent.parent.status.velocity.value
        loop = self.parent.parent.async_lib.library.get_running_loop()
        loop.run_in_executor(
            None, self.parent.parent.driver.movel, [x, y, z, rx, ry, rz], acc, vel
        )

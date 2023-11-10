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
from caproto.server.autosave import autosaved

from .driver import RobotDriver

log = logging.getLogger(__name__)


class ActionsGroup(PVGroup):
    """PVs for RPC actions that the robot can perform."""

    async def run_action(self, action, position):
        """Run an action for a given position along with needed motion
        parameters.

        Assumes the action has a call signature like::

        .. code:: python

            def placel(
                place_goal,
                acc,
                vel,
                gripper_pos_opn,
                gripper_pos_cls,
                gripper_vel,
                gripper_frc,
            )

        """
        # Apply the motion parameters
        status = self.parent.status
        gripper = self.parent.gripper
        params = {
            "acc": status.acceleration.value,
            "vel": status.velocity.value,
            "gripper_pos_opn": gripper.opn.value,
            "gripper_pos_cls": gripper.cls.value,
            "gripper_vel": gripper.vel.value,
            "gripper_frc": gripper.frc.value,
        }
        action_ = partial(action, **params)
        # Execute the action
        loop = self.parent.async_lib.library.get_running_loop()
        return await loop.run_in_executor(None, action_, position)

    @autosaved
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
        return await self.parent.run_action(
            self.parent.parent.driver.pickj, [i, j, k, l, m, n]
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
        return await self.parent.run_action(
            self.parent.parent.driver.pickl, [x, y, z, rx, ry, rz]
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
        return await self.parent.run_action(
            self.parent.parent.driver.placej, [i, j, k, l, m, n]
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
        return await self.parent.run_action(
            self.parent.parent.driver.placel, [x, y, z, rx, ry, rz]
        )

    @autosaved
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
        return await self.parent.run_action(
            self.parent.parent.driver.movej, [i, j, k, l, m, n]
        )

    @autosaved
    @pvfunction(default=[0], prefix="homel:")
    async def homel(
        self,
        x: float = -0.269,
        y: float = 0.105,
        z: float = 0.418,
        rx: float = 2.141,
        ry: float = -2.096,
        rz: float = -0.160,
    ) -> int:
        """Instruct the robot to return to a home position given in Cartesian coordinates."""
        return await self.parent.run_action(
            self.parent.parent.driver.movel, [x, y, z, rx, ry, rz]
        )

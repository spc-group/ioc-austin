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


class TransferGroup(PVGroup):
    # Parameters to the action
    x1 = pvproperty(
        name=".X1", value=0.0, dtype=PvpropertyDouble, doc="Source position x"
    )
    y1 = pvproperty(
        name=".Y1", value=0.0, dtype=PvpropertyDouble, doc="Source position y"
    )
    z1 = pvproperty(
        name=".Z1", value=0.0, dtype=PvpropertyDouble, doc="Source position z"
    )
    x2 = pvproperty(
        name=".X2", value=0.0, dtype=PvpropertyDouble, doc="Target position x"
    )
    y2 = pvproperty(
        name=".Y2", value=0.0, dtype=PvpropertyDouble, doc="Target position y"
    )
    z2 = pvproperty(
        name=".Z2", value=0.0, dtype=PvpropertyDouble, doc="Target position z"
    )
    velocity = pvproperty(name=".VELO", dtype=PvpropertyDouble, doc="Velocity (EGU/s)")
    seconds_to_velocity = pvproperty(
        name=".ACCL",
        dtype=PvpropertyDouble,
        doc="Seconds to Velocity",
        value=0.2,
    )
    engineering_units = pvproperty(
        name=".EGU",
        dtype=PvpropertyChar,
        max_length=16,
        report_as_string=True,
        value="",
        doc="Engineering Units",
    )

    # Status and control fields
    done_moving = pvproperty(
        name=".DMOV",
        dtype=PvpropertyShortRO,
        doc="Done moving to value",
        read_only=True,
        value=1,
    )
    run = pvproperty(
        name=".RUN",
        dtype=PvpropertyShort,
        doc="Direct the robot to start the action",
        value=0,
    )

    @run.putter
    async def run(self, instance, value):
        """Handler for the transfer action on the robot."""
        if not value:
            # Some null value was given, so ignore it
            return
        # Update state PVs
        await self.done_moving.write(0)
        # Prepare arguments to the action
        pos1 = (self.x1.value, self.y1.value, self.z1.value)
        pos2 = (self.x2.value, self.y2.value, self.z2.value)
        action = partial(self.parent.driver.transfer, pos1=pos1, pos2=pos2)
        # Execute the action
        await self.parent.lock()
        try:
            loop = self.parent.async_lib.get_event_loop()
            result = await loop.run_in_executor(None, action)
        finally:
            # Update state PVs
            await self.done_moving.write(1)
            # Release the lock on the robot
            await self.parent.unlock()


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

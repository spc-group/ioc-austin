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


class SampleGroup(PVGroup):

    sample_position: tuple[float] = (0, 0, 0, 0, 0, 0)
    stage_position: tuple[float] = (0, 0, 0, 0, 0, 0)

    def __init__(self, sample_position: tuple[float], stage_position: tuple[float], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sample_position = sample_position
        self.stage_position = stage_position
    
    present = pvproperty(
        name=":present",
        value=False,
        doc="Whether a sample base is present in this platform position",
    )

    load = pvproperty(
        name=":load",
        value=False,
        doc="Direct the robot to load this sample to the stage",
        read_only=False,
    )

    @property
    def is_empty(self):
        result = self.present.value in ["Off", False, 0]
        return result 

    @load.putter
    async def load(self, instance, value):
        """Loads the given sample onto the stage.

        Effectively pick, then home, then place.

        """
        if value != "On":
            return "Off"
        # Check that there is a sample on the platform
        from pprint import pprint
        pprint(dir(instance))
        print(self.is_empty)
        if self.is_empty:
            raise RuntimeError("Sample platform is empty.")
        # Pick the sample up from the platform
        await self.parent.actions.run_action(
            self.parent.driver.pickl, self.sample_position,
        )
        await self.parent.actions.homel.Process.write(1)
        # Place the sample down on the stage
        await self.parent.actions.run_action(
            self.parent.driver.placel, self.stage_position,
        )
        await self.parent.actions.homel.Process.write(1)
        # Update the presence PV
        await self.present.write(0)
        return "Off"

    unload = pvproperty(
        name=":unload",
        value=False,
        doc="Direct the robot to return this sample from the stage to the platform",
        read_only=False,
    )

    @unload.putter
    async def unload(self, instance, value):
        """Unloads the given sample from the stage to the platform.

        Effectively pick, then home, then place.

        """
        if value != "On":
            return
        # Check that there isn't already a sample on the platform
        if not self.is_empty:
            raise RuntimeError("Sample platform is not empty.")
        # Pick the sample up from the stage
        await self.parent.actions.run_action(
            self.parent.driver.pickl, self.stage_position,
        )
        await self.parent.actions.homel.Process.write(1)
        # Move the sample back to the platform
        await self.parent.actions.run_action(
            self.parent.driver.placel, self.sample_position,
        )
        await self.parent.actions.homel.Process.write(1)
        # Update the presence PV
        await self.present.write(1)
        return "Off"

    x = pvproperty(
        name=":x",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    y = pvproperty(
        name=":y",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    z = pvproperty(
        name=":z",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    rx = pvproperty(
        name=":rx",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )
    ry = pvproperty(
        name=":ry",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )
    rz = pvproperty(
        name=":rz",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )

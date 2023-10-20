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

    pick_position: tuple[float] = (0, 0, 0, 0, 0, 0)
    place_position: tuple[float] = (0, 0, 0, 0, 0, 0)

    def __init__(self, pick_position: tuple[float], place_position: tuple[float], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pick_position = pick_position
        self.place_position = place_position
    
    present = pvproperty(
        name="present",
        value=False,
        doc="Whether the sensor detects a sample stage is present",
    )

    load = pvproperty(
        name="load"
        value=False,
        doc="Direct the robot to load this sample to the stage",
        read_only=True,
    )

    @load.putter
    async def load(self, instance, value):
        """Loads the given sample onto the stage.

        Effectively pick, then home, then place.

        """
        if value == "On":
            print(f"Picking the sample from {self.pick_position}")
            print(f"Homing...")
            print(f"Placing the sample at {self.place_position}")
        return "Off"

    x = pvproperty(
        name="x",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    y = pvproperty(
        name="y",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    z = pvproperty(
        name="z",
        value=0.0,
        doc="Translation coordinate of the sample pick position",
        read_only=True,
    )
    rx = pvproperty(
        name="rx",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )
    ry = pvproperty(
        name="ry",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )
    rz = pvproperty(
        name="rz",
        value=0.0,
        doc="Rotation coordinate of the sample pick position",
        read_only=True,
    )

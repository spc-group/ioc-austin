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


POLL_TIME = 5  # Seconds


class GripperGroup(PVGroup):
    """PVs for controller the robot's gripper hand."""

    act = pvproperty(
        name=".ACT", dtype=bool, value="Off", doc="Whether the gripper is activated"
    )

    @act.scan(POLL_TIME)
    async def act(self, instance, async_lib):
        print("Checking for changes to activated status.")

    cls = pvproperty(
        name=".CLS",
        dtype=float,
        value=0,
        doc="Calibrated 'closed' position",
        read_only=True,
    )

    @cls.scan(POLL_TIME)
    async def cls(self, instance, async_lib):
        print("Checking for changes to closed calibration position.")

    opn = pvproperty(
        name=".OPN",
        dtype=float,
        value=0,
        doc="Calibrated 'open' position",
        read_only=True,
    )

    @opn.scan(POLL_TIME)
    async def opn(self, instance, async_lib):
        print("Checking for changes to open calibration position.")

    cal = pvproperty(
        name=".CAL",
        dtype=bool,
        value="Off",
        doc="Calibrate the robot's open/closed range",
    )

    @cal.putter
    async def cal(self, instance, value):
        if value == "On":
            # Launch the calibration script here
            print("Starting gripper calibration")
        return "Off"

    rbv = pvproperty(
        name=".RBV",
        dtype=float,
        value=0,
        doc="Current gripper position readback value",
        read_only=True,
    )

    @rbv.scan(POLL_TIME)
    async def rbv(self, instance, async_lib):
        print("Checking for changes to current gripper position.")

    val = pvproperty(
        name=".VAL", dtype=float, value=0, doc="Desired position set point"
    )

    @val.putter
    async def val(self, instance, value):
        # Launch the calibration script here
        print(f"Moving the gripper to {value}")

    vel = pvproperty(
        name=".VEL", dtype=float, value=0, doc="How fast the gripper should move"
    )
    frc = pvproperty(
        name=".FRC", dtype=float, value=0, doc="How much force the gripper may apply"
    )

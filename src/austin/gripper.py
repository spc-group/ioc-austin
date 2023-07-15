#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
from typing import Literal, Union, OrderedDict
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
        async_lib = self.driver.gripper_act_status
        print("Checking for changes to activated status.")

    @act.putter
    async def act(self, instance, value):
        if value == "On":
            self.driver.activate_gripper
            print("Activating the gripper")
        elif value == "Off":
            self.driver.disconnect_gripper
            print("Deactivating the gripper")
        return "Off"

    cls = pvproperty(
        name=".CLS",
        dtype=float,
        value=0,
        doc="Calibrated 'closed' position",
        read_only=True,
    )

    @cls.scan(POLL_TIME)
    async def cls(self, instance, async_lib):
        value = self.driver.gripper_cls_position
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
        async_lib = self.driver.gripper_cls_position
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
            self.driver.gripper_cal
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
        return self.driver.gripper_cur_position

    val = pvproperty(
        name=".VAL", dtype=float, value=0, doc="Desired position set point"
    )

    vel = pvproperty(
        name=".VEL", dtype=float, value=0, doc="How fast the gripper should move"
    )
    
    frc = pvproperty(
        name=".FRC", dtype=float, value=0, doc="How much force the gripper may apply"
    )

    @val.putter
    async def val(self, instance, value):
        position = value
        speed = self.vel.value
        force = self.frc.value
        self.driver.gripper_move(position, speed, force)
        print(f"Moving the gripper to {value}")
        
        

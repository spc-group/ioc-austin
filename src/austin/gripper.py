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
        name=".ACT", value="Off", dtype=bool, doc="Request that the gripper activate."
    )

    @act.startup
    async def act(self, instance, async_lib):
        self.async_lib = async_lib

    @act.putter
    async def act(self, instance, value):
        if value == "On":
            self.parent.driver.activate_gripper()
            print("Activating the gripper")
        return "Off"

    act_rbv = pvproperty(
        name=".ACR",
        value="Off",
        dtype=bool,
        doc="Whether the gripper is activated",
        read_only=True,
    )

    @act_rbv.scan(POLL_TIME)
    async def act_rbv(self, instance, async_lib):
        new_value = self.parent.driver.gripper_act_status()
        print(new_value)
        if new_value != instance.value:
            await instance.write(new_value)

    cls = pvproperty(
        name=".CLS",
        dtype=int,
        value=0,
        doc="Calibrated 'closed' position",
        read_only=True,
    )

    @cls.scan(POLL_TIME)
    async def cls(self, instance, async_lib):
        new_value = self.parent.driver.gripper_cls_position()
        if new_value != instance.value:
            await instance.write(new_value)

    opn = pvproperty(
        name=".OPN",
        dtype=int,
        value=0,
        doc="Calibrated 'open' position",
        read_only=True,
    )

    @opn.scan(POLL_TIME)
    async def opn(self, instance, async_lib):
        new_value = self.parent.driver.gripper_opn_position()
        if new_value != instance.value:
            await instance.write(new_value)
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
            self.parent.driver.gripper_cal()
            print("Starting gripper calibration")
        return "Off"

    rbv = pvproperty(
        name=".RBV",
        dtype=int,
        value=0,
        doc="Current gripper position readback value",
        read_only=True,
    )

    @rbv.scan(POLL_TIME)
    async def rbv(self, instance, async_lib):
        new_value = self.parent.driver.gripper_cur_position()
        if new_value != instance.value:
            await instance.write(new_value)
        print("Checking for changes to current gripper position.")

    val = pvproperty(name=".VAL", dtype=int, value=0, doc="Desired position set point")

    vel = pvproperty(
        name=".VEL", dtype=int, value=127, doc="How fast the gripper should move"
    )

    frc = pvproperty(
        name=".FRC", dtype=int, value=127, doc="How much force the gripper may apply"
    )

    @val.putter
    async def val(self, instance, value):
        velv = self.vel.value
        frcv = self.frc.value
        loop = self.async_lib.library.get_running_loop()
        await loop.run_in_executor(
            None, self.parent.driver.gripper_move, value, velv, frcv
        )

#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock

# Fixeds a bug in the urx library or dependency
import collections

collections.Iterable = collections.abc.Iterable

from caproto import ChannelType, ChannelDouble
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
    records,
)
from caproto.server.records import MotorFields
from caproto.server.autosave import autosaved

from .driver import RobotDriver


log = logging.getLogger(__name__)


POLL_TIME = 0.5


class RobotAxisFields(MotorFields):
    def __init__(self, *args, axis_num: int = 99, **kwargs):
        self.axis_num = axis_num
        super().__init__(*args, **kwargs)

    @property
    def driver(self):
        # Find the root of the IOC
        obj = self
        while True:
            # Find the next node up the tree
            try:
                parent = obj.parent
            except AttributeError:
                parent = obj.group
            # See if the next node up is the root
            if parent is None:
                # It's the root node, so quit
                break
            else:
                obj = parent
        return obj.driver

    async def tweak_value(self, instance, value, direction):
        """Tweak the motor value. To be used by tweak forward and reverse.

        *direction* should be either 1 (forward) or -1 (reverse).
        """
        # Putting 0 does nothing
        if not bool(value):
            return 0
        # Figure out where to move to
        step = direction * instance.group.tweak_step_size.value
        axis_num = self.parent.axis_num
        # Do the actual moving
        log.info(f"Tweaking axis {axis_num} value by {step}.")
        new_pos = tuple((step if n == axis_num else 0) for n in range(6))
        loop = self.async_lib.library.get_running_loop()
        do_mov = partial(
            self.driver.movel,
            pos=new_pos,
            vel=0.5,
            acc=0.5,
            relative=True,
        )
        await loop.run_in_executor(None, do_mov)

    @MotorFields.tweak_motor_forward.putter
    async def tweak_motor_forward(self, instance, value):
        await self.tweak_value(instance, value, direction=1)
        return 0

    @MotorFields.tweak_motor_reverse.putter
    async def tweak_motor_reverse(self, instance, value):
        await self.tweak_value(instance, value, direction=-1)
        return 0

    @MotorFields.description.startup
    async def description(self, instance, async_lib):
        # Save the async lib for later use
        self.async_lib = async_lib
        # Set the fields to the PV spec properties
        await instance.write(self.parent.__doc__)


class RobotAxisPosition(PvpropertyDouble):
    def __init__(self, *args, axis_num: int = 99, **kwargs):
        self.axis_num = axis_num
        super().__init__(*args, **kwargs)


class StatusGroup(PVGroup):
    async def move_joint(self, instance, value):
        """Callback to move to a given set of joint coordinates.

        Reads target joint position from PVs:

        - i
        - j
        - k
        - l
        - m
        - n

        """
        # Build new coordinate position
        new_joints = []
        for axis in ["i", "j", "k", "l", "m", "n"]:
            if instance is getattr(self, axis):
                new_joints.append(value)
            else:
                new_joints.append(getattr(self, f"{axis}_rbv").value)
        acceleration = self.acceleration.value
        velocity = self.velocity.value
        # Move to the new joint position
        loop = self.async_lib.library.get_running_loop()
        do_mov = partial(
            self.parent.driver.movej,
            joints=new_joints,
            acc=acceleration,
            vel=velocity,
        )
        await loop.run_in_executor(None, do_mov)
        return value

    async def move_position(self, instance, value):
        """Callback to move to a given set of Cartesian coordinates.

        Reads position from PVs:

        - x
        - y
        - z
        - rx
        - ry
        - rz

        """
        # Build new coordinate position
        new_pos = []
        for axis in ["x", "y", "z", "rx", "ry", "rz"]:
            if instance is getattr(self, axis):
                new_pos.append(value)
            else:
                new_pos.append(getattr(self, f"{axis}_rbv").value)
        acceleration = self.acceleration.value
        velocity = self.velocity.value
        # Move to the new joint position
        loop = self.async_lib.library.get_running_loop()
        do_move = partial(
            self.parent.driver.movel, pos=new_pos, acc=acceleration, vel=velocity
        )
        await loop.run_in_executor(None, do_move)
        return value

    # Joint positions
    i = pvproperty(
        name="i",
        value=0.0,
        doc="Position of the first joint",
        put=move_joint,
        precision=3,
    )
    j = pvproperty(
        name="j",
        value=0.0,
        doc="Position of the second joint",
        put=move_joint,
        precision=3,
    )
    k = pvproperty(
        name="k",
        value=0.0,
        doc="Position of the third joint",
        put=move_joint,
        precision=3,
    )
    l = pvproperty(
        name="l",
        value=0.0,
        doc="Position of the fourth joint",
        put=move_joint,
        precision=3,
    )
    m = pvproperty(
        name="m",
        value=0.0,
        doc="Position of the fifth joint",
        put=move_joint,
        precision=3,
    )
    n = pvproperty(
        name="n",
        value=0.0,
        doc="Position of the sixth joint",
        put=move_joint,
        precision=3,
    )
    # Joint position read-back-values
    i_rbv = pvproperty(
        name="i.RBV",
        value=0.0,
        doc="Read-back Position of the 1st joint",
        precision=3,
    )
    j_rbv = pvproperty(
        name="j.RBV",
        value=0.0,
        doc="Read-back Position of the 2nd joint",
        precision=3,
    )
    k_rbv = pvproperty(
        name="k.RBV",
        value=0.0,
        doc="Read-back Position of the 3rd joint",
        precision=3,
    )
    l_rbv = pvproperty(
        name="l.RBV",
        value=0.0,
        doc="Read-back Position of the 4th joint",
        precision=3,
    )
    m_rbv = pvproperty(
        name="m.RBV",
        value=0.0,
        doc="Read-back Position of the 5th joint",
        precision=3,
    )
    n_rbv = pvproperty(
        name="n.RBV",
        value=0.0,
        doc="Read-back Position of the 6th joint",
        precision=3,
    )

    @i_rbv.scan(POLL_TIME)
    async def i_rbv(self, instance, async_lib):
        """Ask the driver for current join positions and update the PVs."""
        loop = self.async_lib.library.get_running_loop()
        # Get current join positions
        new_joints = await loop.run_in_executor(None, self.parent.driver.get_joints)
        # Update PVs with new joint positions
        pvs = [self.i_rbv, self.j_rbv, self.k_rbv, self.l_rbv, self.m_rbv, self.n_rbv]
        for pv, val in zip(pvs, new_joints):
            await pv.write(val)

    @i_rbv.startup
    async def i_rbv(self, instance, async_lib):
        # Just here to get the async lib
        self.async_lib = async_lib

    # Cartesian positions
    x = pvproperty(
        name="x",
        value=0.0,
        doc="Cartesian x",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=0,
        units="m",
        precision=3,
    )
    y = pvproperty(
        name="y",
        value=0.0,
        doc="Cartesian y",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=1,
        units="m",
        precision=3,
    )
    z = pvproperty(
        name="z",
        value=0.0,
        doc="Cartesian z",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=2,
        units="m",
        precision=3,
    )
    rx = pvproperty(
        name="rx",
        value=0.0,
        doc="X-rotation",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=3,
        units="m",
        precision=3,
    )
    ry = pvproperty(
        name="ry",
        value=0.0,
        doc="Y-rotation",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=4,
        units="m",
        precision=3,
    )
    rz = pvproperty(
        name="rz",
        value=0.0,
        doc="Z-rotation",
        put=move_position,
        record=RobotAxisFields,
        dtype=RobotAxisPosition,
        axis_num=5,
        units="m",
        precision=3,
    )

    @x.scan(POLL_TIME)
    async def x(self, instance, async_lib):
        """Ask the driver for current Cartesian position and update the PVs."""
        loop = self.async_lib.library.get_running_loop()
        # Get current position
        new_pos = await loop.run_in_executor(None, self.parent.driver.get_position)
        # Update PVs with new joint positions
        pvs = [
            self.x.fields["RBV"],
            # self.x_rbv,
            self.y.fields["RBV"],
            self.z.fields["RBV"],
            self.rx.fields["RBV"],
            self.ry.fields["RBV"],
            self.rz.fields["RBV"],
        ]
        for pv, val in zip(pvs, new_pos):
            await pv.write(val)

    # Motion parameters
    acceleration = autosaved(
        pvproperty(
            name="acceleration",
            value=0.5,
            doc="Acceleration of the robot joints when starting to move.",
            precision=2,
            record=records.AiFields,
        )
    )
    velocity = autosaved(
        pvproperty(
            name="velocity",
            value=0.2,
            doc="Rotational velocity of the robot joints when moving.",
            precision=2,
            record=records.AiFields,
        )
    )

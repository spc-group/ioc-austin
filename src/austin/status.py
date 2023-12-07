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

    async def move_axis(self, instance, new_pos, vel, acc, relative):
        # Indicate that the axis is moving
        await instance.group.motor_is_moving.write(1)
        await instance.group.done_moving_to_value.write(0)
        # Do the move in a separate thread
        loop = self.async_lib.library.get_running_loop()
        do_mov = partial(
            self.do_move,
            new_pos=new_pos,
            vel=vel,
            acc=acc,
            relative=relative,
        )
        await loop.run_in_executor(None, do_mov)
        # Indicate that the axis is done
        await instance.group.motor_is_moving.write(0)
        await instance.group.done_moving_to_value.write(1)

    def do_move(self, new_pos, vel, acc, relative):
        """A stub that decides what moving this axis means.

        Intended to be easily overwritten by subclasses
        (e.g. RobotJointFields).

        """
        self.driver.movel(new_pos, vel=vel, acc=acc, relative=relative)

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
        # Decide how fast to move
        acceleration = instance.group.jog_accel.value
        velocity = instance.group.jog_velocity.value
        # Do the actual moving
        log.info(f"Tweaking axis {axis_num} value by {step}.")
        new_pos = tuple((step if n == axis_num else 0) for n in range(6))
        await self.move_axis(
            instance, new_pos, vel=velocity, acc=acceleration, relative=True
        )

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

    @MotorFields.jog_accel.startup
    async def jog_accel(self, instance, async_lib):
        """Set the jog accel and velocity to sensible values
        
        This is a hack, these should really be autosaved."""
        await self.jog_accel.write(0.2)
        await self.jog_velocity.write(0.5)


class RobotJointFields(RobotAxisFields):
    def do_move(self, new_pos, vel, acc, relative):
        self.driver.movej(new_pos, vel=vel, acc=acc, relative=relative)


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
                new_joints.append(getattr(self, f"{axis}").fields["RBV"].value)
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
                new_pos.append(getattr(self, f"{axis}").fields["RBV"].value)
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
            doc="Base",
            put=move_joint,
            record=RobotJointFields,
            dtype=RobotAxisPosition,
            axis_num=0,
            units="rad",
            precision=3,
        )
    j = pvproperty(
        name="j",
        value=0.0,
        doc="Shoulder",
        put=move_joint,
        record=RobotJointFields,
        dtype=RobotAxisPosition,
        axis_num=1,
        units="rad",
        precision=3,
    )
    k = pvproperty(
        name="k",
        value=0.0,
        doc="Elbow",
        put=move_joint,
        record=RobotJointFields,
        dtype=RobotAxisPosition,
        axis_num=2,
        units="rad",
        precision=3,
    )
    l = pvproperty(
        name="l",
        value=0.0,
        doc="Wrist 1",
        put=move_joint,
        record=RobotJointFields,
        dtype=RobotAxisPosition,
        axis_num=3,
        units="rad",
        precision=3,
    )
    m = pvproperty(
        name="m",
        value=0.0,
        doc="Wrist 2",
        put=move_joint,
        record=RobotJointFields,
        dtype=RobotAxisPosition,
        axis_num=4,
        units="rad",
        precision=3,
    )
    n = pvproperty(
        name="n",
        value=0.0,
        doc="Wrist 3",
        put=move_joint,
        record=RobotJointFields,
        dtype=RobotAxisPosition,
        axis_num=5,
        units="rad",
        precision=3,
    )

    @i.scan(POLL_TIME)
    async def i(self, instance, async_lib):
        """Ask the driver for current join positions and update the PVs."""
        loop = self.async_lib.library.get_running_loop()
        # Get current join positions
        new_joints = await loop.run_in_executor(None, self.parent.driver.get_joints)
        # Update PVs with new joint positions
        pvs = [
            self.i.fields["RBV"],
            self.j.fields["RBV"],
            self.k.fields["RBV"],
            self.l.fields["RBV"],
            self.m.fields["RBV"],
            self.n.fields["RBV"],
        ]
        # pvs = [self.i_rbv, self.j_rbv, self.k_rbv, self.l_rbv, self.m_rbv, self.n_rbv]
        for pv, val in zip(pvs, new_joints):
            await pv.write(val)

    @i.startup
    async def i(self, instance, async_lib):
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

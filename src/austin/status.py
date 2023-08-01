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
)

from .driver import RobotDriver


POLL_TIME = 50


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
    )
    j = pvproperty(
        name="j",
        value=0.0,
        doc="Position of the second joint",
        put=move_joint,
    )
    k = pvproperty(
        name="k",
        value=0.0,
        doc="Position of the third joint",
        put=move_joint,
    )
    l = pvproperty(
        name="l",
        value=0.0,
        doc="Position of the fourth joint",
        put=move_joint,
    )
    m = pvproperty(
        name="m",
        value=0.0,
        doc="Position of the fifth joint",
        put=move_joint,
    )
    n = pvproperty(
        name="n",
        value=0.0,
        doc="Position of the sixth joint",
        put=move_joint,
    )
    # Joint position read-back-values
    i_rbv = pvproperty(
        name="i.RBV", value=0.0, doc="Read-back Position of the 1st joint"
    )
    j_rbv = pvproperty(
        name="j.RBV", value=0.0, doc="Read-back Position of the 2nd joint"
    )
    k_rbv = pvproperty(
        name="k.RBV", value=0.0, doc="Read-back Position of the 3rd joint"
    )
    l_rbv = pvproperty(
        name="l.RBV", value=0.0, doc="Read-back Position of the 4th joint"
    )
    m_rbv = pvproperty(
        name="m.RBV", value=0.0, doc="Read-back Position of the 5th joint"
    )
    n_rbv = pvproperty(
        name="n.RBV", value=0.0, doc="Read-back Position of the 6th joint"
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
    x = pvproperty(name="x", value=0.0, doc="Cartesian position x", put=move_position)
    y = pvproperty(name="y", value=0.0, doc="Cartesian position y", put=move_position)
    z = pvproperty(name="z", value=0.0, doc="Cartesian position z", put=move_position)
    rx = pvproperty(
        name="rx", value=0.0, doc="Rotation around the x-axis", put=move_position
    )
    ry = pvproperty(
        name="ry", value=0.0, doc="Rotation around the y-axis", put=move_position
    )
    rz = pvproperty(
        name="rz", value=0.0, doc="Rotation around the z-axis", put=move_position
    )
    # Cartesian read-back values
    x_rbv = pvproperty(
        name="x.RBV", value=0.0, doc="Read-back position of the x coordinate"
    )
    y_rbv = pvproperty(
        name="y.RBV", value=0.0, doc="Read-back position of the y coordinate"
    )
    z_rbv = pvproperty(
        name="z.RBV", value=0.0, doc="Read-back position of the z coordinate"
    )
    rx_rbv = pvproperty(
        name="rx.RBV", value=0.0, doc="Read-back rotation of the x coordinate"
    )
    ry_rbv = pvproperty(
        name="ry.RBV", value=0.0, doc="Read-back rotation of the y coordinate"
    )
    rz_rbv = pvproperty(
        name="rz.RBV", value=0.0, doc="Read-back rotation of the z coordinate"
    )

    @x_rbv.scan(POLL_TIME)
    async def x_rbv(self, instance, async_lib):
        """Ask the driver for current Cartesian position and update the PVs."""
        loop = self.async_lib.library.get_running_loop()
        # Get current position
        new_pos = await loop.run_in_executor(None, self.parent.driver.get_position)
        # Update PVs with new joint positions
        pvs = [
            self.x_rbv,
            self.y_rbv,
            self.z_rbv,
            self.rx_rbv,
            self.ry_rbv,
            self.rz_rbv,
        ]
        for pv, val in zip(pvs, new_pos):
            await pv.write(val)

    # Motion parameters
    acceleration = pvproperty(
        name="acceleration",
        value=0.5,
        doc="Acceleration of the robot joints when starting to move.",
    )
    velocity = pvproperty(
        name="velocity",
        value=0.2,
        doc="Rotational velocity of the robot joints when moving.",
    )

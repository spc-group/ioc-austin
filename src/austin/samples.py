#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
from typing import Literal, Sequence, Tuple
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
    """Describes a sample holder sitting on the shelf.

    The robot will be able to retrieve the sample holder from its position,
    and place it on the stage (and also vice-versa).

    When going from shelf to the stage, it will go through the points
    specified in *waypoints* in order. When going from the stage back to the shelf,
    it will go through *waypoints* in reverse order.

    Parameters
    ----------
    sample_position
      Cartesian coordinates (x, y, z, rx, ry, rz) of the sample holder on the shelf.
    stage_position
      Cartesian coordinates (x, y, z, rx, ry, rz) of the sample holder on the translation stage.
    waypoints
      Cartesian coordinates (x, y, z, rx, ry, rz) sets to follow on the way from the shelf to the stage.

    """

    sample_position: tuple[float] = (0, 0, 0, 0, 0, 0)
    stage_position: tuple[float] = (0, 0, 0, 0, 0, 0)
    forward_path: Sequence[Tuple[float]] = []

    def __init__(
        self,
        sample_position: tuple[float],
        stage_position: tuple[float],
        waypoints: tuple[float],
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.sample_position = sample_position
        self.stage_position = stage_position
        self.waypoints = waypoints

    present = pvproperty(
        name=":present",
        value=True,
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
            self.parent.driver.pickl,
            self.sample_position,
        )
        # Go through each waypoint, in order, to move to the stage
        for waypoint in self.waypoints:
            await self.parent.actions.run_action(self.parent.driver.movel, waypoint)
        # Place the sample down on the stage
        await self.parent.actions.run_action(
            self.parent.driver.placel,
            self.stage_position,
        )
        # Go back to a safe position (really the first waypoint)
        for waypoint in waypoints[::-1]:
            await self.parent.actions.run_action(self.parent.driver.movel, waypoint)
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
        # Go through each waypoint, in order, to move to the stage
        for waypoint in self.waypoints:
            await self.parent.actions.run_action(self.parent.driver.movel, waypoint)
        # Pick the sample up from the stage
        await self.parent.actions.run_action(
            self.parent.driver.pickl,
            self.stage_position,
        )
        # Go back to the waypoint closest to the stage
        for waypoint in waypoints[::-1]:
            await self.parent.actions.run_action(self.parent.driver.movel, waypoint)
        # Move the sample back to the platform
        await self.parent.actions.run_action(
            self.parent.driver.placel,
            self.sample_position,
        )
        # Go back to the first waypoint position (resting position)
        await self.parent.actions.run_action(self.parent.driver.movel, waypoints[0])
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

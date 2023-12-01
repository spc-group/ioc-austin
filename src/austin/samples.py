#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
from typing import Literal, Sequence, Tuple
import enum

import numpy as np
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


# Aerotech stage
xyzStage = [0.20354, -0.41995, -0.06055]
# xyzStage = [0.23074, -0.43944, -0.06051]
# RxyzStage = [2.893, 1.226, 0.003]
RxyzStage = [-2.893, -1.226, -0.003]
posStage_del = [0.00499, 0.00105, 0, 0, 0, 0]

stage_position = np.array(xyzStage + RxyzStage) + np.array(posStage_del)

# 24 Holders
xyz8 = [-0.07709, 0.35717, 0.06101]
xyz22 = [0.07552, 0.20068, 0.06088]

x_del = (xyz22[0] - xyz8[0]) / 2
y_del = (xyz8[1] - xyz22[1]) / 2

pos15 = (np.array(xyz8) + np.array(xyz22)) / 2
pos0 = [pos15[0] - 3 * x_del, pos15[1] + 2 * y_del, pos15[2]]

Rxyz0 = [-2.244, 2.2, 0.009]


sample_positions = []
for n in range(24):
    x = pos0[0] + n % 6 * x_del
    y = pos0[1] - n // 6 * y_del
    z = pos15[2]
    pos = [x, y, z] + Rxyz0
    sample_positions.append(pos)

# A trajectory that the robot can follow to safely navigate from the shelf to the stage
shelf_to_stage_path = [
    [-0.26948, 0.10508, 0.41812],
    [-0.04074, -0.40701, 0.31061],
]


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
    waypoints: Sequence[Tuple[float]] = []

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

    empty = pvproperty(
        name=":empty",
        value=True,
        doc="Whether a sample base is not present in this platform position",
    )

    @present.scan(0.1)
    async def present(self, instance, async_lib):
        """Ask the attached labjacks whether a sample is present."""
        bases_per_labjack = 12
        start_lj_position = 8
        lj_num = int(self.sample_num / bases_per_labjack)
        lj_pos = self.sample_num % bases_per_labjack + start_lj_position
        # Get the PV property for the sensor of this sample
        lj_group = getattr(self.parent.parent, f"labjack{lj_num}")
        dio = getattr(lj_group.digital_ios, f"dio{lj_pos}")
        lj_val = dio.input.value
        # Convert from labjack values to binary values
        if lj_val == "Low":
            present_val = "Off"
            empty_val = "On"
        else:
            present_val = "On"
            empty_val = "Off"
            # Write the new presence switch value to the PV
        if present_val != instance.value:
            await instance.write(present_val)
            await self.empty.write(empty_val)

        # print(f"Sample {self.sample_num} is on LJ {lj_num} position {lj_pos}: {new_val}")

    load = pvproperty(
        name=":load",
        value=False,
        doc="Direct the robot to load this sample to the stage",
        read_only=False,
    )

    @property
    def driver(self):
        return self.parent.parent.driver

    @property
    def actions(self):
        return self.parent.parent.actions

    @property
    def is_empty(self):
        result = self.present.value in ["Off", False, 0]
        return result

    @property
    def sample_num(self):
        return int(self.prefix.split(":")[-1][6:])

    @load.putter
    async def load(self, instance, value):
        """Loads the given sample onto the stage.

        Effectively pick, then home, then place.

        """
        if value != "On":
            return "Off"
        # Check that there is a sample on the platform
        if self.is_empty:
            raise RuntimeError("Sample platform is empty.")
        # Check if we need to unload an existing sample first
        await self.parent.unload_current_sample.write("On")
        # Pick the sample up from the platform
        await self.actions.run_action(
            self.driver.pickl,
            self.sample_position,
        )
        # Go through each waypoint, in order, to move to the stage
        for waypoint in self.waypoints:
            await self.actions.run_action(self.driver.movel, waypoint)
        # Place the sample down on the stage
        await self.actions.run_action(
            self.driver.placel,
            self.stage_position,
        )
        # Update the current sample PV for later unloading
        print(f"Set current_sample to {self.sample_num}")
        await self.parent.current_sample.write(str(self.sample_num))
        # Go back to a safe position (really the first waypoint)
        for waypoint in self.waypoints[::-1]:
            await self.actions.run_action(self.driver.movel, waypoint)
        # Update the presence PV
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
            await self.actions.run_action(self.driver.movel, waypoint)
        # Pick the sample up from the stage
        await self.actions.run_action(
            self.driver.pickl,
            self.stage_position,
        )
        # Go back to the waypoint closest to the stage
        for waypoint in self.waypoints[::-1]:
            await self.actions.run_action(self.driver.movel, waypoint)
        # Move the sample back to the platform
        await self.actions.run_action(
            self.driver.placel,
            self.sample_position,
        )
        # Update the current sample position PV
        await self.parent.current_sample.write("None")
        # Go back to the first waypoint position (resting position)
        await self.actions.run_action(self.driver.movel, self.waypoints[0])
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


class SamplesGroup(PVGroup):
    """Covers all the sample positions, and keeps track of which sample is loaded."""

    current_sample = autosaved(
        pvproperty(name="current_sample", value="None", dtype=str, record="stringin")
    )
    unload_current_sample = pvproperty(
        name="unload_current_sample", value=False, dtype=bool
    )

    @unload_current_sample.putter
    async def unload_current_sample(self, instance, value):
        """Remove the current sample from the stage, if one is present."""
        # Check that the value given should start the robot
        if value != "On":
            return "Off"
        # Ask the current sample to remove itself from the stage
        current_sample = self.current_sample.value
        if current_sample in ["None", None]:
            log.info("Current sample is `None`, ignoring unload request.")
        else:
            await getattr(self, f"sample{current_sample}").unload.write("On")
        return "Off"

    # sample0 = SubGroup(
    # SampleGroup,
    # prefix="sample0",
    # sample_position=sample_positions[0],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample1 = SubGroup(
    # SampleGroup,
    # prefix="sample1",
    # sample_position=sample_positions[1],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample2 = SubGroup(
    # SampleGroup,
    # prefix="sample2",
    # sample_position=sample_positions[2],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample3 = SubGroup(
    # SampleGroup,
    # prefix="sample3",
    # sample_position=sample_positions[3],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample4 = SubGroup(
    # SampleGroup,
    # prefix="sample4",
    # sample_position=sample_positions[4],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample5 = SubGroup(
    # SampleGroup,
    # prefix="sample5",
    # sample_position=sample_positions[5],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample6 = SubGroup(
    # SampleGroup,
    # prefix="sample6",
    # sample_position=sample_positions[6],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample7 = SubGroup(
    # SampleGroup,
    # prefix="sample7",
    # sample_position=sample_positions[7],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    sample8 = SubGroup(
        SampleGroup,
        prefix="sample8",
        sample_position=sample_positions[8],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample9 = SubGroup(
        SampleGroup,
        prefix="sample9",
        sample_position=sample_positions[9],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample10 = SubGroup(
        SampleGroup,
        prefix="sample10",
        sample_position=sample_positions[10],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    # sample11 = SubGroup(
    # SampleGroup,
    # prefix="sample11",
    # sample_position=sample_positions[11],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample12 = SubGroup(
    # SampleGroup,
    # prefix="sample12",
    # sample_position=sample_positions[12],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample13 = SubGroup(
    # SampleGroup,
    # prefix="sample13",
    # sample_position=sample_positions[13],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    sample14 = SubGroup(
        SampleGroup,
        prefix="sample14",
        sample_position=sample_positions[14],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample15 = SubGroup(
        SampleGroup,
        prefix="sample15",
        sample_position=sample_positions[15],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample16 = SubGroup(
        SampleGroup,
        prefix="sample16",
        sample_position=sample_positions[16],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    # sample17 = SubGroup(
    # SampleGroup,
    # prefix="sample17",
    # sample_position=sample_positions[17],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample18 = SubGroup(
    # SampleGroup,
    # prefix="sample18",
    # sample_position=sample_positions[18],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    # sample19 = SubGroup(
    # SampleGroup,
    # prefix="sample19",
    # sample_position=sample_positions[19],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )
    sample20 = SubGroup(
        SampleGroup,
        prefix="sample20",
        sample_position=sample_positions[20],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample21 = SubGroup(
        SampleGroup,
        prefix="sample21",
        sample_position=sample_positions[21],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample22 = SubGroup(
        SampleGroup,
        prefix="sample22",
        sample_position=sample_positions[22],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    # sample23 = SubGroup(
    # SampleGroup,
    # prefix="sample23",
    # sample_position=sample_positions[23],
    # stage_position=stage_position,
    # waypoints=shelf_to_stage_path,
    # )

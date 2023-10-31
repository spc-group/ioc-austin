#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
import numpy as np
from functools import partial
from threading import Lock

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
    autosave,
)
from caprotoapps import AliveGroup

from .driver import RobotDriver
from .dashboard import DashboardGroup
from .status import StatusGroup
from .actions import ActionsGroup
from .gripper import GripperGroup
from .samples import SampleGroup

log = logging.getLogger(__name__)


class RobotBusy(RuntimeError):
    """The robot cannot be controlled because it is currently busy."""

    ...


# Aerotech stage
xyzStage = [0.20354, -0.41995, -0.06055]
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
    [-0.04074, -0.40701, 0.31061],
    [-0.26948, 0.10508, 0.41812],
]


class AustinIOC(PVGroup):
    """An input output controller for the 25-ID-C universal robots sample
    changing robot.

    """

    _lock = Lock()

    # Robot-related PVs
    status = SubGroup(StatusGroup, prefix="")
    actions = SubGroup(ActionsGroup, prefix="")
    dashboard = SubGroup(DashboardGroup, prefix="dashboard")
    gripper = SubGroup(GripperGroup, prefix="gripper")
    busy = pvproperty(
        name="busy",
        value=False,
        doc="Whether the global run lock is being held.",
        read_only=True,
    )

    # Sample loaders
    sample0 = SubGroup(
        SampleGroup,
        prefix="sample0",
        sample_position=sample_positions[0],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample1 = SubGroup(
        SampleGroup,
        prefix="sample1",
        sample_position=sample_positions[1],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample2 = SubGroup(
        SampleGroup,
        prefix="sample2",
        sample_position=sample_positions[2],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample3 = SubGroup(
        SampleGroup,
        prefix="sample3",
        sample_position=sample_positions[3],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample4 = SubGroup(
        SampleGroup,
        prefix="sample4",
        sample_position=sample_positions[4],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample5 = SubGroup(
        SampleGroup,
        prefix="sample5",
        sample_position=sample_positions[5],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample6 = SubGroup(
        SampleGroup,
        prefix="sample6",
        sample_position=sample_positions[6],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample7 = SubGroup(
        SampleGroup,
        prefix="sample7",
        sample_position=sample_positions[7],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
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
    sample11 = SubGroup(
        SampleGroup,
        prefix="sample11",
        sample_position=sample_positions[11],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample12 = SubGroup(
        SampleGroup,
        prefix="sample12",
        sample_position=sample_positions[12],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample13 = SubGroup(
        SampleGroup,
        prefix="sample13",
        sample_position=sample_positions[13],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
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
    sample17 = SubGroup(
        SampleGroup,
        prefix="sample17",
        sample_position=sample_positions[17],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample18 = SubGroup(
        SampleGroup,
        prefix="sample18",
        sample_position=sample_positions[18],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
    sample19 = SubGroup(
        SampleGroup,
        prefix="sample19",
        sample_position=sample_positions[19],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )
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
    sample23 = SubGroup(
        SampleGroup,
        prefix="sample23",
        sample_position=sample_positions[23],
        stage_position=stage_position,
        waypoints=shelf_to_stage_path,
    )

    # Support PVs
    autosave = SubGroup(autosave.AutosaveHelper)
    alive = SubGroup(AliveGroup, prefix="alive", remote_host="xapps2.xray.aps.anl.gov")

    def __init__(
        self, robot_ip, robot_port=29999, gripper_port=63352, timeout=5, *args, **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.driver = RobotDriver(
            robot_ip=robot_ip,
            robot_port=robot_port,
            gripper_port=gripper_port,
            timeout=timeout,
        )

    async def __ainit__(self, async_lib):
        self.async_lib = async_lib
        # Note that we have to pass this in to ``run()``!

    async def lock(self):
        """Prevent other actions for happening on the robot.

        Raises
        ======
        RobotBusy
          Another program action already has a lock on the robot.

        """
        result = self._lock.acquire(blocking=False)
        if result is False:
            # Lock is not available, raise exception
            raise RobotBusy("Another action is already being executed on this robot.")
        else:
            await self.busy.write(1)

    async def unlock(self):
        """Allow other actions to happen on the robot again."""
        self._lock.release()
        await self.busy.write(0)

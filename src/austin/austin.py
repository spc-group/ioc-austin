#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
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


sample_positions = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 0
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 1
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 2
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 3
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 4
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 5
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 6
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 7
    (-.06858, 0.35933, -0.10418+0.4, 2.215, -2.230, 0.009), # 8
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 9
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 10
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 11
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 12
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 13
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 14
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 15
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 16
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 17
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 18
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 19
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 20
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 21
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 22
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), # 23
]

stage_position = (0.19676, -0.42392, -0.22204+0.4, 2.909, 1.187, 0.003)  # Aerotech stage


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
        SampleGroup, prefix="sample0",
        sample_position=sample_positions[0],
        stage_position=stage_position,
    )
    sample8 = SubGroup(
        SampleGroup, prefix="sample8",
        sample_position=sample_positions[8],
        stage_position=stage_position,
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

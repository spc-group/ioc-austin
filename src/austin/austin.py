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
from caprotoapps import AliveGroup, LabJackT4

from .driver import RobotDriver
from .dashboard import DashboardGroup
from .status import StatusGroup
from .actions import ActionsGroup
from .gripper import GripperGroup
from .samples import SamplesGroup

log = logging.getLogger(__name__)


class RobotBusy(RuntimeError):
    """The robot cannot be controlled because it is currently busy."""

    ...


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
    samples = SubGroup(SamplesGroup, prefix="")

    # Support PVs
    autosave = SubGroup(autosave.AutosaveHelper)
    alive = SubGroup(AliveGroup, prefix="alive", remote_host="xapps2.xray.aps.anl.gov")

    # Labjacks for responding to changes in the sample presence switches
    labjack0 = SubGroup(LabJackT4, prefix="LJT4_0:", identifier="labjackAustin00")
    labjack1 = SubGroup(LabJackT4, prefix="LJT4_1:", identifier="labjackAustin01")

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

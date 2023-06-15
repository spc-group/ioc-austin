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
)

from .driver import RobotDriver
from .dashboard import DashboardGroup
from .status import StatusGroup
from .actions import TransferGroup, ActionsGroup
from .alive import AliveGroup

log = logging.getLogger(__name__)


class RobotBusy(RuntimeError):
    """The robot cannot be controlled because it is currently busy."""
    ...


class RobotIOC(PVGroup):
    _lock = Lock()

    robot = SubGroup(StatusGroup, prefix="robot")
    transfer = SubGroup(TransferGroup, prefix="transfer")
    actions = SubGroup(ActionsGroup, prefix="")
    dashboard = SubGroup(DashboardGroup, prefix="dashboard")
    alive = SubGroup(AliveGroup, prefix="alive")

    def __init__(self, robot_ip, port=29999, timeout=5, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.driver = RobotDriver(robot_ip=robot_ip, port=port, timeout=timeout)

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
            await self.robot.busy.write(1)

    async def unlock(self):
        """Allow other actions to happen on the robot again."""
        self._lock.release()
        await self.robot.busy.write(0)

#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock

from caproto import ChannelType
from caproto.server import PVGroup, ioc_arg_parser, pvproperty, run, PvpropertyDouble, PvpropertyShort, PvpropertyShortRO, PvpropertyChar, SubGroup

from .driver import RobotDriver


log = logging.getLogger(__name__)


position_names = ["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"]


class RobotBusy(RuntimeError):
    """The robot cannot be controlled because it is currently busy."""
    ...


class StatusGroup(PVGroup):
    busy = pvproperty(name=":busy", value=False, doc="If the robot is busy.", read_only=True)
    mood = pvproperty(name=":mood", value="Bored", doc="How does the robot feel right now?", read_only=True)

    @mood.scan(period=0.1)
    async def mood(self, instance, async_lib):
        """
        Scan hook for getting the robot's mood.

        Parameters
        ----------
        instance : ChannelData
            This is the instance of ``my_property``.

        async_lib : AsyncLibraryLayer
            This is a shim layer for {asyncio, curio, trio} that you can use
            to make async library-agnostic IOCs.
        """
        new_mood = self.parent.driver.mood()
        if new_mood != instance.value:
            await instance.write(new_mood)


class TransferGroup(PVGroup):
    # Parameters to the action
    x1 = pvproperty(name=".X1", value=0., dtype=PvpropertyDouble, doc="Source position x")
    y1 = pvproperty(name=".Y1", value=0., dtype=PvpropertyDouble, doc="Source position y")
    z1 = pvproperty(name=".Z1", value=0., dtype=PvpropertyDouble, doc="Source position z")
    x2 = pvproperty(name=".X2", value=0., dtype=PvpropertyDouble, doc="Target position x")
    y2 = pvproperty(name=".Y2", value=0., dtype=PvpropertyDouble, doc="Target position y")
    z2 = pvproperty(name=".Z2", value=0., dtype=PvpropertyDouble, doc="Target position z")
    velocity = pvproperty(
        name=".VELO", dtype=PvpropertyDouble, doc="Velocity (EGU/s)"
    )
    seconds_to_velocity = pvproperty(
        name=".ACCL",
        dtype=PvpropertyDouble,
        doc="Seconds to Velocity",
        value=0.2,
    ) 
    engineering_units = pvproperty(
        name=".EGU",
        dtype=PvpropertyChar,
        max_length=16,
        report_as_string=True,
        value="",
        doc="Engineering Units",
    )

    # Status and control fields
    done_moving = pvproperty(
        name=".DMOV",
        dtype=PvpropertyShortRO,
        doc="Done moving to value",
        read_only=True,
        value=1,
    )
    run = pvproperty(
        name=".RUN",
        dtype=PvpropertyShort,
        doc="Direct the robot to start the action",
        value=0,
    )

    @run.putter
    async def run(self, instance, value):
        """Handler for the transfer action on the robot."""
        if not value:
            # Some null value was given, so ignore it
            return
        # Update state PVs
        await self.done_moving.write(0)
        # Prepare arguments to the action
        pos1 = (self.x1.value, self.y1.value, self.z1.value)
        pos2 = (self.x2.value, self.y2.value, self.z2.value)
        action = partial(self.parent.driver.transfer, pos1=pos1, pos2=pos2)
        # Execute the action
        await self.parent.lock()
        try:
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, action)
        finally:
            # Update state PVs
            await self.done_moving.write(1)
            # Release the lock on the robot
            await self.parent.unlock()    


class RobotIOC(PVGroup):
    _lock = Lock()

    robot = SubGroup(StatusGroup, prefix="robot")
    transfer = SubGroup(TransferGroup, prefix="transfer")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.driver = RobotDriver()

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


def main():
    ioc_options, run_options = ioc_arg_parser(
        default_prefix='25idAustin:',
        desc='Run an IOC that operates the sample changing robot.')

    # Instantiate the IOC, assigning a prefix for the PV names.
    ioc = RobotIOC(**ioc_options)

    # Run IOC.
    run(ioc.pvdb, **run_options)


if __name__ == '__main__':
    sys.exit(main())

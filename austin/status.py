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

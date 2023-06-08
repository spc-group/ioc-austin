#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
import re

from caproto import ChannelType
from caproto.server import PVGroup, pvproperty, PvpropertyDouble, PvpropertyShort, PvpropertyShortRO, PvpropertyChar, SubGroup, scan_wrapper

from .driver import RobotDriver


POLL_TIME = 0.5


class RobotCommandFailed(RuntimeError):
    ...


class DashboardGroup(PVGroup):

    lock: asyncio.Lock = None

    async def send_command(self, message: str, fmt: str = None):
        """Ask the robot for information via a serial command.

        Parameters
        ==========
        message
          Message to send to the robot to ask for information.
        fmt
          Regular expression format string to parse the response .

        Returns
        =======
        new_value
          The answer replied by the robot.
        """
        # Prepare the socket lock if necessary
        if self.lock is None:
            self.lock = self.parent.async_lib.library.Lock()
        # Send the message in a separate thread
        loop = self.parent.async_lib.library.get_running_loop()
        async with self.lock:
            reply = await loop.run_in_executor(
                None, self.parent.driver.send_and_receive, message)
        # Parse the response
        if fmt is not None:
            value = re.match(fmt, reply)
            if value is None:
                raise RobotCommandFailed(reply)
            else:
                return value.group(1)
    
    remote_control = pvproperty(name=":remote_control", value=False,
                                doc="Whether the robot is in remote control mode.", read_only=True)

    @remote_control.scan(POLL_TIME)
    async def remote_control(self, instance, async_lib):
        new_val = await self.send_command("is in remote control", fmt="(.+)")
        new_val = (new_val == "true")
        if new_val != instance.value:
            await instance.write(new_val)

    current_program = pvproperty(name=":program_rbv", value="whatever", dtype=str,
                         read_only=True,
                         doc="Name of the currently loaded program.")

    @current_program.scan(POLL_TIME)
    async def current_program(self, instance, async_lib):
        new_val = await self.send_command("get loaded program", fmt="Loaded program: (.+)")
        if new_val != instance.value:
            await instance.write(new_val)

    program = pvproperty(name=":program", value=" ", dtype=str,
                         doc="Change to a new loaded program.")
    
    @program.putter
    async def program(self, instance, value):
        await self.send_command(f"load {value}", fmt="Loading program: (.+)")

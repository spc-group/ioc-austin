#!/usr/bin/env python3
import logging
import sys
import time
import asyncio
from functools import partial
from threading import Lock
import re
import enum

from caproto import ChannelType, SkipWrite
from caproto.server import (
    PVGroup,
    pvproperty,
    PvpropertyDouble,
    PvpropertyShort,
    PvpropertyShortRO,
    PvpropertyChar,
    SubGroup,
    scan_wrapper,
)

from .driver import RobotDriver, RobotDisconnected

POLL_SLOWDOWN = 1  # Should be 1 for normal operation, high for debugging
POLL_TIME = 0.5 * POLL_SLOWDOWN


def reset_trigger(value):
    return False


class RobotCommandFailed(RuntimeError):
    ...


class SafetyStatus(enum.IntEnum):
    UNKNOWN = 0
    NORMAL = 1
    REDUCED = 2
    PROTECTIVE_STOP = 3
    RECOVERY = 4
    SAFEGUARD_STOP = 5
    SYSTEM_EMERGENCY_STOP = 6
    ROBOT_EMERGENCY_STOP = 7
    VIOLATION = 8
    FAULT = 9
    AUTOMATIC_MODE_SAFEGUARD_STOP = 10
    SYSTEM_THREE_POSITION_ENABLING_STOP = 11


class RobotMode(enum.IntEnum):
    UNKNOWN = 0
    NO_CONTROLLER = 1
    DISCONNECTED = 2
    CONFIRM_SAFETY = 3
    BOOTING = 4
    POWER_OFF = 5
    POWER_ON = 6
    IDLE = 7
    BACKDRIVE = 8
    RUNNING = 9


class ProgramState(enum.IntEnum):
    UNKNOWN = 0
    STOPPED = 1
    PLAYING = 2
    PAUSED = 3


class OperationalMode(enum.IntEnum):
    MANUAL = 0
    AUTOMATIC = 1
    NONE = 2


class DashboardGroup(PVGroup):
    lock: asyncio.Lock = None

    async def send_command(self, message: str, fmt: str = None):
        """Ask the robot for information via a serial command.

        Parameters
        ==========
        message
          Message to send to the robot to ask for information.
        fmt
          Regular expression format string to parse the response.

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
            try:
                reply = await loop.run_in_executor(
                    None, self.parent.driver.send_and_receive, message
                )
            except RobotDisconnected:
                return None
        # Parse the response
        if fmt is not None:
            value = re.match(fmt, reply)
            if value is None:
                await self.message.write(reply)
                raise RobotCommandFailed(reply)
            elif len(value.groups()) > 0:
                return value.group(1)

    async def put_command(self, instance, value, message, fmt, convert=lambda x: None):
        """Send a command to perform some change on the robot's state."""
        message = message.format(value=value)
        print(f"Sending command: ``{message}``")
        updated_val = await self.send_command(message, fmt=fmt)
        print(f"Received response: {updated_val}")
        updated_val = convert(updated_val)
        return updated_val

    async def scan_command(self, instance, async_lib, message, fmt, convert=None):
        """Send a request for status update, and return the parsed value."""
        new_val = await self.send_command(message, fmt=fmt)
        # Convert
        if convert is not None:
            new_val = convert(new_val)
        if new_val is not None and new_val != instance.value:
            await instance.write(new_val)

    # Define PVs below

    remote_control = pvproperty(
        name=":remote_control",
        value=False,
        doc="Whether the robot is in remote control mode.",
        read_only=True,
    )
    remote_control = remote_control.scan(POLL_TIME)(
        partial(
            scan_command,
            message="is in remote control",
            fmt="(.+)",
            convert=lambda x: x == "true",
        )
    )

    current_program = pvproperty(
        name=":program_rbv",
        value="",
        dtype=str,
        read_only=True,
        report_as_string=True,
        max_length=255,
        doc="Name of the currently loaded program.",
    )
    current_program = current_program.scan(POLL_TIME)(
        partial(scan_command, message="get loaded program", fmt="Loaded program: (.+)")
    )

    program = pvproperty(
        name=":program",
        value="",
        dtype=PvpropertyChar,
        doc="Change to a new loaded program.",
        report_as_string=True,
        max_length=255,
    )
    program = program.putter(
        partial(put_command, message="load {value}", fmt="Loading program: (.+)")
    )
    installation = pvproperty(
        name=":installation",
        value="",
        dtype=PvpropertyChar,
        doc="Change to a new loaded installation file.",
        report_as_string=True,
        max_length=255,
    )
    installation = installation.putter(
        partial(
            put_command,
            message="load installation {value}",
            fmt="Loading installation: (.+)",
        )
    )

    play = pvproperty(
        name=":play",
        value=False,
        dtype=bool,
        doc="Instruct the robot to play the current program.",
        put=partial(
            put_command, message="play", fmt="Starting program", convert=reset_trigger
        ),
    )
    stop = pvproperty(
        name=":stop",
        value=False,
        dtype=bool,
        doc="Instruct the robot to stop the current program.",
        put=partial(put_command, message="stop", fmt="Stopped", convert=reset_trigger),
    )
    pause = pvproperty(
        name=":pause",
        value=False,
        dtype=bool,
        doc="Instruct the robot to pause the current program.",
        put=partial(
            put_command, message="pause", fmt="Pausing program", convert=reset_trigger
        ),
    )
    quit_ = pvproperty(
        name=":quit",
        value=False,
        dtype=bool,
        doc="Instruct the robot to quit.",
        put=partial(
            put_command, message="quit", fmt="Disconnected", convert=reset_trigger
        ),
    )
    shutdown = pvproperty(
        name=":shutdown",
        value=False,
        dtype=bool,
        doc="Instruct the robot to shut down.",
        put=partial(
            put_command, message="shutdown", fmt="Shutting down", convert=reset_trigger
        ),
    )
    release_brake = pvproperty(
        name=":release_brake",
        value=False,
        dtype=bool,
        doc="Instruct the robot to release the brake.",
        put=partial(
            put_command,
            message="brake release",
            fmt="Brake releasing",
            convert=reset_trigger,
        ),
    )
    close_safety_popup = pvproperty(
        name=":close_safety_popup",
        value=False,
        dtype=bool,
        doc="Closes the displayed safety popup.",
        put=partial(
            put_command,
            message="close safety popup",
            fmt="closing safety popup",
            convert=reset_trigger,
        ),
    )
    unlock_protective_stop = pvproperty(
        name=":unlock_protective_stop",
        value=False,
        dtype=bool,
        doc="Closes the current popup and unlocks protective stop.",
        put=partial(
            put_command,
            message="unlock protective stop",
            fmt="Protective stop releasing",
            convert=reset_trigger,
        ),
    )
    restart_safety = pvproperty(
        name=":restart_safety",
        value=False,
        dtype=bool,
        doc="Used when robot gets a safety fault or violation to restart the safety.",
        put=partial(
            put_command,
            message="restart safety",
            fmt="Restarting safety",
            convert=reset_trigger,
        ),
    )
    # Read-only status PVs
    serial_number = pvproperty(
        name=":serial_number",
        value="",
        dtype=PvpropertyChar,
        doc="Serial number of the robot.",
        read_only=True,
        report_as_string=True,
        max_length=255,
        startup=partial(
            scan_command,
            message="get serial number",
            fmt="(.+)",
        ),
    )
    model_number = pvproperty(
        name=":model_number",
        value="unknown",
        dtype=PvpropertyChar,
        doc="Model number of the robot.",
        read_only=True,
        report_as_string=True,
        max_length=255,
        startup=partial(
            scan_command,
            message="get robot model",
            fmt="(.+)",
        ),
    )
    software_version = pvproperty(
        name=":software_version",
        value="unknown",
        dtype=PvpropertyChar,
        doc="Version number of the software installed on the robot.",
        read_only=True,
        report_as_string=True,
        max_length=255,
        startup=partial(
            scan_command,
            message="PolyscopeVersion",
            fmt="(.+)",
        ),
    )
    program_running = pvproperty(
        name=":program_running",
        value=False,
        dtype=bool,
        doc="Whether a program is currently running on the robot.",
        read_only=True,
    )
    program_running = program_running.scan(POLL_TIME)(
        partial(
            scan_command,
            message="running",
            fmt="Program running: (true|false)",
            convert=lambda x: x == "true",
        )
    )
    program_saved = pvproperty(
        name=":program_saved",
        value=False,
        dtype=bool,
        doc="Whether a current program is saved.",
        read_only=True,
    )
    program_saved = program_saved.scan(POLL_TIME)(
        partial(
            scan_command,
            message="isProgramSaved",
            fmt="(true|false) .+",
            convert=lambda x: x == "true",
        )
    )
    # This :safety_status PV should use the SafetyStatus enum, but
    # some of the strings are longer than the 26 character limit, and
    # so this will need to be factored.
    safety_status = pvproperty(
        name=":safety_status",
        value="",
        dtype=str,
        read_only=True,
        report_as_string=True,
        max_length=255,
        doc="Reports the current robot safety stats.",
    )
    safety_status = safety_status.scan(POLL_TIME)(
        partial(
            scan_command,
            message="safetystatus",
            fmt="Safetystatus: (.+)",
        ),
    )
    robot_mode = pvproperty(
        name=":robot_mode",
        value=RobotMode.UNKNOWN,
        read_only=True,
        doc="Reports the current robot operating mode.",
    )

    @robot_mode.scan(POLL_TIME)
    async def robot_mode(self, instance, async_lib):
        new_val = await self.send_command(message="robotmode", fmt="Robotmode: (.+)")
        if new_val is not None and new_val != instance.value:
            await instance.write(new_val)
            # Set the power on/off state based on the new mode
            power_state = getattr(RobotMode, new_val) >= RobotMode.POWER_ON
            await self.power_rbv.write(power_state)

    program_state = pvproperty(
        name=":program_state",
        value=ProgramState.UNKNOWN,
        read_only=True,
        doc="Reports the state of the active program.",
    )
    program_state = program_state.scan(POLL_TIME)(
        partial(
            scan_command,
            message="programState",
            fmt="([A-Z_]+) .+",
        ),
    )

    # PVs for change/reading the power state
    power = pvproperty(
        name=":power", dtype=bool, value=False, doc="Powers the robot arm on and off."
    )

    @power.putter
    async def power(self, instance, value):
        """Turn the robot arm power on or off."""
        if value == "On":
            cmd = "power on"
        elif value == "Off":
            cmd = "power off"
        else:
            raise ValueError(f"Power setting must be 'On' or 'Off'. Got: {value}")
        return await self.put_command(
            instance,
            value,
            message=cmd,
            fmt="Powering (.+)",
            convert=lambda x: x == "on",
        )

    # Read-back value for power state is set during ``robot_mode`` scan
    power_rbv = pvproperty(
        name=":power_rbv",
        value=False,
        dtype=bool,
        read_only=True,
        doc="The current power on/off state.",
    )

    # PVs for change/reading the Robot's operational mode
    operational_mode = pvproperty(
        name=":operational_mode",
        value=OperationalMode.MANUAL,
        doc="The requested operational mode: MANUAL or AUTOMATIC (the password has not been set).",
        put=partial(
            put_command,
            message="set operational mode {value}",
            fmt="Operational mode '(.+)' is set",
        ),
    )
    operational_mode_rbv = pvproperty(
        name=":operational_mode_rbv",
        value=OperationalMode.MANUAL,
        read_only=True,
        doc="The robot's current operational mode: MANUAL, AUTOMATIC, or NONE (the password has not been set)..",
    )
    operational_mode_rbv = operational_mode_rbv.scan(POLL_TIME)(
        partial(
            scan_command,
            message="get operational mode",
            fmt="(.+)",
        ),
    )

    # A generic PV that can be used by other PVs to report an error message
    message = pvproperty(
        name=":message",
        read_only=True,
        dtype=PvpropertyChar,
        value="",
        report_as_string=True,
        max_length=255,
        doc="Most recent dashboard message reported by the robot.",
    )

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
)

from .robot import RobotIOC

log = logging.getLogger(__name__)


ROBOT_IP = "164.54.119.60"
# ROBOT_IP = None


def main():
    ioc_options, run_options = ioc_arg_parser(
        default_prefix="25idAustin:",
        desc="Run an IOC that operates the sample changing robot.",
    )

    # Instantiate the IOC, assigning a prefix for the PV names.
    ioc = RobotIOC(robot_ip=ROBOT_IP, **ioc_options)

    # Run IOC.
    run(ioc.pvdb, startup_hook=ioc.__ainit__, **run_options)


if __name__ == "__main__":
    sys.exit(main())

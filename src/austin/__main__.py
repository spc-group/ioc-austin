import textwrap
import logging

import caproto.server

from .austin import AustinIOC

log = logging.getLogger(__name__)


ROBOT_IP = "164.54.119.60"
# ROBOT_IP = None




def main():
    ioc_options, run_options = caproto.server.ioc_arg_parser(
        default_prefix='25idAustin:',
        desc=textwrap.dedent(AustinIOC.__doc__)
    )

    # Instantiate the IOC, assigning a prefix for the PV names.
    ioc = AustinIOC(robot_ip=ROBOT_IP, **ioc_options)    

    caproto.server.run(ioc.pvdb, startup_hook=ioc.__ainit__, **run_options)


if __name__ == '__main__':
    main()

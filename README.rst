============
 IOC Austin
============

Using the IOC
=============

An IOC for the Austin Universal Robot at sector 25.

To use, install caproto and run the following from this directory:

.. code:: bash
	  
   python -m austin.start_ioc --list-pvs


To interact with the IOC, send the desired position to the
``25idAustin:sample`` process variable (PV), which will run the
necessary robot code (not yet implemented). The PV ``25idAustin:busy``
can be monitored to see when the move is done.

E.g. in one terminal:

.. code:: bash

    camonitor 25idAustin:busy

Then in a second terminal:

.. code:: bash

    caput 25idAustin:sample A2

caproto vs EPICS
================

Since the robot uses python, it makes sense to use control software
written naively in python.

**EPICS** is a c/c++ library that can be used to **build a compiled
binary**, known as an input-output controller (IOC). Once executed,
the IOC will listen on a network port for messages using the
channel-access protocol (CA), and respond to messages based on its
configuration. There is nothing magic about the CA protocol, and it
has also been implemented in other tools, most notably
**caproto**. *caproto* is a python-native CA library.

The IOC developed here uses caproto to listen for CA messages that
will direct it to run python routines for manipulating the robot.

Writing IOC Support
===================

By itself, the IOC does nothing interesting. The actual work is done
by the :py:class:`~austin.driver.RobotDriver` class, using the *urx* library
or similar.

The *urx* library is synchronous, while the IOC is necessarily
asynchronous. For example, the *sample* PV has a handler that calls
*driver.load_sample*. If the handler were to call this synchronous
function directly, then the IOC would be unresponsive until the robot
is done. Crucially, **this would make it impossible to ask the IOC if
it is finished or to abort the current move** if necessary.

To avoid this problem, the handler is an asynchronous coroutine that
runs the robot's methods in a separate thread.

The general approach, therefore, is that each action the robot can
perform has one of more PVs associated with it, and certain of these
PVs have *putter* handlers that deal with the asynchronous transition.

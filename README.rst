============
 IOC Austin
============

Austin **depends on the caproto-apps package** for communicating with
an IOC Alive server. Make sure to install it first: ``pip install
caproto-apps``.

Using the IOC
=============

An IOC for the Austin Universal Robot at sector 25.

To use, install the IOC once with

.. code:: bash

   pip install -e .

Then run it with:

.. code:: bash
	  
   start_25idAustin --list-pvs

Dashboard Commands
------------------

The standard dashboard commands do things like monitor the robot mode,
cycle power on and off, etc. These are available in the
``25idAustin:dashboard`` prefix. E.g. to load and then play a program:

.. code:: bash

    caput 25idAustin:dashboard:program 1sample.urp
    caget 25idAustin:dashboard:program_rbv
    caput 25idAustin:dashboard:play 1

Actions
-------

Robot actions are python functions that are executed by the IOC
(e.g. using the urx library). Each robot action on the IOC has a
prefix, for example ``25idAustin:transfer`` is the prefix for the
*transfer* action. Send the desired positions to the
``25idAustin:transfer.X1`` (and Y1..Z2) process variable (PV), and set
any additional arguments on the transfer PVs. Then caput *1* to the
``25idAustin:transfer.run`` PV, which will run the robot action's
python function (not yet implemented). The PV
``25idAustin:robot:busy`` can be monitored to see when the move is
done.

E.g. in one terminal:

.. code:: bash

    camonitor 25idAustin:robot:busy

Then in a second terminal:

.. code:: bash

    caput 25idAustin:transfer.X1 15.3
    caput 25idAustin:transfer.Y1 22.9
    caput 25idAustin:transfer.Z1 -11.0
    caput 25idAustin:transfer.X2 99.18
    caput 25idAustin:transfer.Y2 -87
    caput 25idAustin:transfer.Z2 2
    caput 25idAustin:transfer.VELO 0.1
    caput 25idAustin:transfer.RUN 1
    
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

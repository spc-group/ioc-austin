============
 IOC Austin
============

A channel-access IOC for the Austin Universal Robot at sector 25,
written in caproto (not EPICS).

This robot IOC is composed of 4 sections:

- `Dashboard Commands`_: Simple commands to control low-level robot features.
- `Status`_: Reports and controls the current position and state of the robot.
- `Gripper`_: Controls the gripper connect to the robot arm for manipulating objects.
- `Actions`_: Allows arbitrary python code to be run for complex actions by the robot.

Additionally, more information is available about using the IOC:

- `A Note on Async/Await`_
- `Extending this IOC`_
- `caproto vs EPICS`_

The files **pv-mappings.org** contains a comprehensive list of the PVs
available on the IOC and their intended function.

A Note on Async/Await
=====================

In order to stay responsive while the robot is in motion, the IOC
relies heavily on python's async/await keywords in order to pass
control back to the main IOC loop while waiting on the robot to finish
moving.

This IOC has adopted the practice of putting all synchronous code in
the file *src/austin/driver.py*, and then keeping the remaining IOC
files as asynchronous as possible. This is merely a convention, the
violation of which may make the IOC more difficult to debug.

The **IOC will not work properly** if slow, synchronous functions are
run directly inside of a putter or scanner (decorated by
``@<name>.scan`` or ``@<name>.putter``). Instead, co-routines should
be used, possibly by wrapping synchronous functions inside a thread to
then be awaited, for example::

.. code:: python

    async def move_position(self, instance, value):
        # Equivalent to self.parent.driver.movej(value)
        await loop.run_in_executor(None, self.parent.driver.movej, value)

Using the IOC
=============

Austin **depends on the caproto-apps package** for communicating with
an IOC Alive server. Make sure to install it first: ``pip install
caproto-apps``.

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

Status
------

The status PVs provide information and control for the overall state
of the robot, such as the positions of individual joints. There are
two sets of PVs for **interacting with the robot's position**, all of
which have the root ``25idAustin:`` prefix.

The first set control the robot's **joints**. For example,
``25idAustin:i`` is the first joint position and ``caput
25idAustin:i 2.5`` will move the joint to the position 2.5, while
``caget 25idAustin:i.RBV`` will report the read-back value for the
first joint. The PV's ``:j`` through ``:n`` represent joints 2
through 6.

The second set controls the robot's position in **lab
coordinates**. These follow the same general pattern as for joint
positions. Their PVs are ``:x``, ``:y``, and ``:z`` for the
translation, and ``:rx``, ``:ry``, and ``:rz`` for the orientation of
the tool. These PVs are converted to joint positions behind the
scenes.

Additionally, the ``25idAustin:acceleration`` and
``25idAustin:velocity`` PVs determine the acceleration and velocity
with which the robot moves.

Lastly, the ``25idAustin:busy`` PV reports whether the IOC is working
on anything at the moment. **The robot cannot report whether it is in
the middle of an operation**, so be aware that this busy PV only
accounts for tasks directed by this IOC.

Gripper
-------

The robot may be fitted with a gripper for interacting with
objects. These PVs control the gripper independently of the rest of
the robot arm. The gripper subgroup also has its own velocity and
force for moving the gripper. See pv-mappings.org for details on the
PVs available for controlling the gripper.

Actions
-------

Robot actions are python functions that are executed by the IOC
(e.g. using the urx library). Each robot action on the IOC has a
prefix, for example ``25idAustin:pick`` is the prefix for the *pick*
action which picks up the object at a given position. Send the desired
positions to the ``25idAustin:transfer.i`` and j..n process
variables (PV), and set any additional arguments on the pick
PVs. Then caput *1* to the ``25idAustin:transfer.Process`` PV, which will
run the robot action's python function (not yet implemented). The PV
``25idAustin:busy`` can be monitored to see when the actions is done.

E.g. in one terminal:

.. code:: bash

    camonitor 25idAustin:busy

Then in a second terminal:

.. code:: bash

    caput 25idAustin:pick.i 15.3
    caput 25idAustin:pick.j 22.9
    caput 25idAustin:pick.k -11.0
    caput 25idAustin:pick.l 99.18
    caput 25idAustin:pick.m -87
    caput 25idAustin:pick.n 2
    caput 25idAustin:pick.Process 1

Extending this IOC
==================

Each deployed robot will likely need to perform unique tasks, and so
it is likely that this IOC will be extended. The best place to start
is in ``src/austin/actions.py``. Each action here should encapsulated
to a single function. Caproto provides the ``@pvfunction`` decorator
to convert the function signature to PVs for the arguments, plus
extras for processing the function, retrieving the return value, and
monitoring its status. As before, the function provided to
``@pvfunction`` must be awaitable (i.e. use the ``async`` keyword),
and any long-running, synchronous functions called should be run in a
separate executor.

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

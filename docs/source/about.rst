===========
 IOC Austin
===========

A channel-access IOC for the Austin Universal Robot at sector 25,
written in caproto (not EPICS).

This robot IOC is composed of 4 sections:

#. :ref:`Dashboard Commands`: Simple commands to control low-level robot features.
#. :ref:`Status`: Reports and controls the current position and state of the robot.
#. :ref:`Gripper`: Controls the gripper connect to the robot arm for manipulating objects.
#. :ref:`Actions`: Allows arbitrary python code to be run for complex actions by the robot.


Additionally, more information is available about using the IOC:

* :ref:`A Note on Async Await` 
* :ref:`Extending this IOC`
* :ref:`caproto vs EPICS`

The files **pv-mappings.org** contains a comprehensive list of the PVs
available on the IOC and their intended function.

.. _A Note on Async Await:

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

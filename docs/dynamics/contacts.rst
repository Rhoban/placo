Contacts
========

Fixed contact
-------------

Unilateral contact
------------------

Fixed point contact
-------------

Point contact
-------------

Puppet contact
--------------

External wrench contact
-----------------------

Generic task contact
--------------------

Fixing the floating base
------------------------

Assuming a robot is fixed (no floating base), you can use the :func:`mask_fbase() <placo.DynamicsSolver.mask_fbase>`
method to disable the floating base:

.. code-block:: python

    # Disable the floating base
    solver.mask_fbase(True)

This will add a constraint on the floating base that should have no acceleration, and allow forces to be applied
by the floating base to compensate for bias forces such as gravity.

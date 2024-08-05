Contacts
========

.. admonition:: What are contacts ?

    Contacts model the constraints imposed by the **interaction with physical environment**.
    For example, the fact that a foot can't penetrate the ground, or can't (easily) slide on it is a contact.

    In some sense, contacts are **tasks that are taken care of by the physics**, instead of the robot.
    To do so, the environment applies additional forces to the robot.

In PlaCo, any task that is actually handled by physical constraints can be associated with a contact.
To do so, the solver will add **extra forces** as decision variables.

However, physical constraints sometime don't handle all the aspects of a task.
Even if a foot can't penetrate the ground, it can still lift up, no force will prevent it
(this is an unilateral contact).
Also, because of the friction coefficient, the ground can't prevent the foot from sliding, this
depends on the relation between the normal force and thet tangential force.
For those reasons, some specific contacts are implemented in PlaCo.



Module placo_utils
==================

Visualization
-------------

The following methods can be used as helpers to render objects in *Meshcat*:

.. automodule:: placo_utils.visualization
    :members:
    :undoc-members:

TF
---

Importing:

.. code-block:: python

    import placo_utils.tf as tf

Actually is an alias to `Meshcat transformations <https://github.com/meshcat-dev/meshcat-python/blob/master/src/meshcat/transformations.py>`_, containing straightforward implementations to handle rigid body transformations.

The typical two methods you want to use are `rotation_matrix` and `translation_matrix`:

.. code-block:: python

    # Rotation of 0.5 rad around the z axis
    T = tf.rotation_matrix(0.5, [0, 0, 1])

    # Translation of 1m along the x axis
    T = tf.translation_matrix([1, 0, 0])
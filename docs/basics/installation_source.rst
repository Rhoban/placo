Installation from source
========================

PlaCo is mostly written in C++, relying on Boost Python to expose the C++ code to Python.
If you want to get the sources yourself, you will first have to retrieve the dependencies and then to build
the library.

System dependencies
~~~~~~~~~~~~~~~~~~~

You can install system dependencies using:

.. code-block:: bash

    bash scripts/requirements.sh

From the repository.

.. admonition:: Note

  The above script will setup apt-repository for `robotpkg <https://stack-of-tasks.github.io/pinocchio/download.html>`_, and add extra configurations in your ``.bashrc`` file. You might have to reload your bash after.


.. admonition:: Instaling on older systems

  Above scenario was tested on Ubuntu 22.04. If you want to build on an older version, you might need to adapt the following:

  * Adapt Python's version in above `apt` command (e.g. replace ``py310`` by ``py38`` for Python 3.8)
  * Install `Eigen <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_ with at least version 3.4.0

Cloning and building
~~~~~~~~~~~~~~~~~~~~

You can now clone PlaCo:

.. code-block:: bash

    git clone https://www.github.com/rhoban/placo.git

And build it using CMake:

.. code-block:: bash

    cd placo
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j 8

Using built Pythin bindings
~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can now add the ``build/lib/python3-X/site-packages`` directory to your ``PYTHONPATH`` to use PlaCo.

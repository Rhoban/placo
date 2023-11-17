From source
===========

PlaCo is mostly written in C++, relying on Boost Python to expose the C++ code to Python.
If you want to get the sources yourself, you will first have to retrieve the dependencies and then to build
the library.

System dependencies
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo apt-get install python3-dev libpython3-dev libboost-python-dev \
             doxygen libjsoncpp-dev

Pinocchio dependencies
~~~~~~~~~~~~~~~~~~~~~~

First, follow this
`procedure <https://stack-of-tasks.github.io/pinocchio/download.html>`_
to add the apt repositories for Pinocchio.

You can then install the following dependencies:

.. code-block:: bash

    sudo apt-get install robotpkg-hpp-fcl robotpkg-eiquadprog \
            robotpkg-pinocchio robotpkg-py310-pinocchio

.. note::
    Change ``robotpkg-py310-pinocchio`` according to your own Python version.

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

You can now add the ``build/lib/python3/dist-packages`` directory to your ``PYTHONPATH`` to use PlaCo.

Installation from source
========================

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

    sudo apt-get install \
            robotpkg-py310-eigenpy=3.1.1 \
            robotpkg-hpp-fcl=2.3.6 \
            robotpkg-pinocchio=2.6.20 \
            robotpkg-py310-eigenpy=3.1.1  \
            robotpkg-py310-hpp-fcl=2.3.6 \
            robotpkg-py310-pinocchio=2.6.20 \
            robotpkg-eiquadprog

.. note::
    Versions above are frozen to ensure compatiblity. Placo has not yet switched to Pinocchio 3.

  .. note::
    If you use a different version of Python, you will have to adapt the version of the packages
    (e.g. replace ``py310`` by ``py38`` for Python 3.8).

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

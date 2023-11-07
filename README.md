# Rhoban Planning and Control (placo)

## Setting up dependencies

First, install `placo` in a workspace using `wks`:

    pip install wks
    wks install rhoban/placo

To build, you will need:

* Pinocchio [https://stack-of-tasks.github.io/pinocchio/download.html]
  * `robotpkg-py38-pinocchio` (adapt the Python version)
  * `robotpkg-pinocchio`
  * `robotpkg-eiquadprog`
  * `robotpkg-hpp-fcl`
* The following apt repositories:
  * `python3-dev libpython3-dev libboost-python1.71.0 doxygen libjsoncpp-dev`

## Running the build

Simply run:

`wks build`

*Note: at the end of the build, the libraries will end up in `build/lib`. the `stubs.py` script will be run to generate
the `placo.pyi` file that contains more useful informations for autocompletions*

## Using Python bindings

You will need to add `build/lib` to your `PYTHONPATH` to be able to use `placo` library.

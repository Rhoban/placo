# Generate Doxygen stubs 

This code allow to generate a `module.pyi` file containing useful auto-completion stubs, based on Doxygen and introspection.

## Step1: configure your project

In `config.py`, set the name of the module and the path to the source directory (where the `Doxyfile` is)

## Step2: expose through registry

Include `registry.h` in your Python bindings, and use `class__` instead of `class_`, so that the registered classes will be wrapped in the registry

## Step3: call `stubs.py` after compilation

Execute the `stubs.py` script after compilation, it will output the `pyi` file.
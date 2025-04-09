#!/bin/bash

export PATH=$(getconf PATH)
export PKG_CONFIG_PATH=$(getconf PKG_CONFIG_PATH)
export LD_LIBRARY_PATH=$(getconf LD_LIBRARY_PATH)
export PYTHONPATH="."
export CMAKE_PREFIX_PATH=$(getconf CMAKE_PREFIX_PATH)

python -m build --sdist --wheel

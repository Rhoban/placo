#!/bin/bash

INTERPRETER=$1
MODULE_DIRECTORY=$2

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export PYTHONPATH="$MODULE_DIRECTORY:$PYTHONPATH"
$INTERPRETER $SCRIPT_DIR/stubs.py
#!/bin/bash

# Installing doxystub
pip install doxystub

# APT dependencies
sudo apt install -qqy lsb-release curl libboost-python-dev libjsoncpp-dev doxygen python-is-python3

UNAME_M=`uname -m`
if [ "$UNAME_M" == "aarch64" ]; then
    echo "Robotpkg is not available for aarch64 architecture, skipping installation"
    exit
else
    # Installing robotpkg
    bash scripts/robotpkg.sh

    # Installing robotpkg dependencies
    sudo apt install -qqy robotpkg-py3*-pinocchio robotpkg-coal robotpkg-eiquadprog robotpkg-*-eigenpy
fi
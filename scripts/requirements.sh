#!/bin/bash

# Installing doxystub
pip install doxystub

# APT dependencies
sudo apt install -qqy lsb-release curl libboost-python-dev libjsoncpp-dev doxygen

# Installing robotpkg
bash scripts/robotpkg.sh

# Installing robotpkg dependencies
sudo apt install -qqy robotpkg-py3*-pinocchio robotpkg-coal robotpkg-eiquadprog robotpkg-*-eigenpy
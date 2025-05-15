#!/bin/bash

# Installing doxystub
pip install doxystub

# APT dependencies
sudo apt install -qqy lsb-release curl libboost-python-dev libjsoncpp-dev

# Installing robotpkg
bash scripts/robotpkg.sh
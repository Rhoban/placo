#!/bin/bash

# Installing robotpkg
# see https://stack-of-tasks.github.io/pinocchio/download.html
if [ ! -f /etc/apt/sources.list.d/robotpkg.list ]
then
  echo "* Setting up robotpkg apt..."
  sudo mkdir -p /etc/apt/keyrings
  curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
      | sudo tee /etc/apt/keyrings/robotpkg.asc

  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list

  sudo apt update
else
  echo "* robotpkg is already present"
fi

PATH_TEST=`echo $PATH|grep openrob|wc -l`
if [ $PATH_TEST -eq 0 ]
then
  echo "Adding environment variables to .bashrc: you might have to reload your shell"

  PYTHON_VERSION=`python --version|cut -d" " -f2|cut -d"." -f-2`

  echo "# Openrobot packages" >> ~/.bashrc
  echo "export PATH=/opt/openrobots/bin:\$PATH" >> ~/.bashrc
  echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> ~/.bashrc
  echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
  echo "export PYTHONPATH=/opt/openrobots/lib/python$PYTHON_VERSION/site-packages:\$PYTHONPATH" >> ~/.bashrc
  echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
fi
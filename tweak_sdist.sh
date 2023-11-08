#!/bin/bash

# Moving to dist/
cd dist/

# Retrieving package name (e.g: placo-X.Y.Z)
package=`basename *.tar.gz .tar.gz`
echo ${package}

# Unpacking the wheel
wheel unpack *.whl

# Retrieving the placo.pyi produced in the wheel and copying it to the root
placo_pyi=`find ${package}/ -name "placo.pyi"`
cp ${placo_pyi} ${package}/

# Adding it to the tar.gz
gunzip ${package}.tar.gz
tar -rf ${package}.tar ${package}/placo.pyi
gzip ${package}.tar

rm -rf ${package}

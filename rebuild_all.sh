#!/bin/bash
set -e 

cd gtsam/build 

make -j4

sudo make install 

pwd 

cd ../../gtsam/build/python

python setup.py install --force 

cd ../../../

cd build

make -j4 

sudo make install 

sudo make python-install 

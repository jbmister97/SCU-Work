#!/bin/bash

echo "Setting up environmental variables..."

SYSTEMC_HOME="/home/justin/Documents/systemc-2.3.3"

export SYSTEMC=$SYSTEMC_HOME
export LD_LIBRARY_PATH=$SYSTEMC_HOME/lib-linux64


echo $SYSTEMC
echo $LD_LIBRARY_PATH

echo "Environmental variables created..."

#!/bin/bash

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
THIS_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$THIS_DIR/setup.sh"

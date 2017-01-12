#!/usr/bin/zsh

export CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
THIS_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R zsh -c 'source "$THIS_DIR/setup.sh"'

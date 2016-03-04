#!/bin/sh
#
# [WARNING]: This file is the entry point for ssh connections used by ROS.
#   - This file is intended to only be called by ROS's machine env-loader.
#   - Tipically, the $SHELL used for this is not "bash", so in order to mantain
#     compatibility within bender framework across machines, you should only use
#     POSIX shell functionalities.

# check invalid usage
if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# mark the entry point as an SSH connection
export BENDER_NET_BY_SSH="YES"

# bender framework as usual
. "$HOME"/bender.sh

# execute remote call
exec "$@"

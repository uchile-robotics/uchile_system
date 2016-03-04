#!/bin/sh

##############################################################################################
#   bender system defs
##############################################################################################

# update package list
rospack profile >/dev/null 2>/dev/null

# bender_* pkg lists
_pkgs_and_paths=$(rospack list | grep "bender_.* ")
export BENDER_PACKAGES_BASE="$(echo "$_pkgs_and_paths" | grep ".*base_ws.*" | sed 's/ [^.*]*//')"
export BENDER_PACKAGES_SOFT="$(echo "$_pkgs_and_paths" | grep ".*soft_ws.*" | sed 's/ [^.*]*//')"
export BENDER_PACKAGES_HIGH="$(echo "$_pkgs_and_paths" | grep ".*high_ws.*" | sed 's/ [^.*]*//')"
export BENDER_PACKAGES="$(printf "%s\n%s\n%s" "$BENDER_PACKAGES_BASE" "$BENDER_PACKAGES_SOFT" "$BENDER_PACKAGES_HIGH")"


##############################################################################################
#   bender system tools
##############################################################################################

# sh utilities named with "_bender_" preffix
. "$BENDER_SYSTEM"/bash/functions.sh

# utilities for building with cmake and ROS
#. "$BENDER_SYSTEM"/bash/make_tools.sh

# utilities useful when working on bash/sh
. "$BENDER_SYSTEM"/bash/tools.sh



# if using bash
if [ "$CATKIN_SHELL" = "bash" ]; then

    # common autocompletion
    . "$BENDER_SYSTEM"/bash/complete.bash

    # tools autocompletion
    #. "$BENDER_SYSTEM"/bash/make_tools_complete.bash
    . "$BENDER_SYSTEM"/bash/tools_complete.bash

fi




##############################################################################################
#   source setup.sh files
##############################################################################################


# source .sh files
for _pkg in $BENDER_PACKAGES
do
    # related setup.sh file
    _pkg_path="$(echo "$_pkgs_and_paths" | grep "$_pkg " | sed 's/^.* //')"
    _setup_file="$_pkg_path"/bash/setup.sh

    # source if neccesary.
    if [ -f "$_setup_file" ]; then
        . "$_setup_file"
    fi

done
unset _pkgs_and_paths
unset _pkg
unset _pkg_path
unset _setup_file


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

# bender_* stacks lists
export BENDER_STACKS=$(rosstack list-names | grep "bender_")


##############################################################################################
#   bender system tools
##############################################################################################
# the source order is important!, at least functions.sh must be placed first.

# sh utilities named with "_bender_" preffix
. "$BENDER_SYSTEM"/shell/functions.sh

# utilities for building with cmake and ROS
. "$BENDER_SYSTEM"/shell/make_tools.sh

# utilities useful when working on bash/sh
. "$BENDER_SYSTEM"/shell/tools.sh

# utilities for doing git stuff on the whole workspace
. "$BENDER_SYSTEM"/shell/gittools.sh


# autocomplete
if _bender_check_if_bash_or_zsh ; then

    # common autocompletion
    . "$BENDER_SYSTEM"/shell/complete/common.sh

    # tools autocompletion
    . "$BENDER_SYSTEM"/shell/complete/tools.sh
    . "$BENDER_SYSTEM"/shell/complete/gittools.sh

fi



##############################################################################################
#   source setup.sh files
##############################################################################################


# source .sh files
for _pkg in $BENDER_PACKAGES
do
    # related setup.sh file
    _pkg_path="$(echo "$_pkgs_and_paths" | grep "$_pkg " | sed 's/^.* //')"
    _setup_file="$_pkg_path"/shell/setup.sh

    # source if neccesary.
    if [ -f "$_setup_file" ]; then
        . "$_setup_file"
    fi

done
unset _pkgs_and_paths
unset _pkg
unset _pkg_path
unset _setup_file


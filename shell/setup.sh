#!/bin/sh

# update package list
rospack profile >/dev/null 2>/dev/null

# pkg lists
_pkgs_and_paths=$(rospack list | grep "$ROBOT_WS")
export ROBOT_PACKAGES_BASE="$(echo "$_pkgs_and_paths" | grep ".*base_ws.*" | sed 's/ [^.*]*//')"
export ROBOT_PACKAGES_SOFT="$(echo "$_pkgs_and_paths" | grep ".*soft_ws.*" | sed 's/ [^.*]*//')"
export ROBOT_PACKAGES_HIGH="$(echo "$_pkgs_and_paths" | grep ".*high_ws.*" | sed 's/ [^.*]*//')"
export ROBOT_PACKAGES="$(printf "%s\n%s\n%s" "$ROBOT_PACKAGES_BASE" "$ROBOT_PACKAGES_SOFT" "$ROBOT_PACKAGES_HIGH")"

# bender_* stacks lists
export ROBOT_STACKS="$(rosstack list | grep $ROBOT_WS | cut -d' ' -f1)"

##############################################################################################
#   bender system tools
##############################################################################################
# the source order is important!, at least functions.sh must be placed first.

# sh utilities named with "_bender_" preffix
. "$ROBOT_SYSTEM"/shell/functions.sh

# utilities useful when working on bash/sh
. "$ROBOT_SYSTEM"/shell/tools.sh

# utilities for doing git stuff on the whole workspace
. "$ROBOT_SYSTEM"/shell/gittools.sh


# autocomplete
if _bender_check_if_bash_or_zsh ; then

    # common autocompletion
    . "$ROBOT_SYSTEM"/shell/complete/common.sh

    # tools autocompletion
    . "$ROBOT_SYSTEM"/shell/complete/tools.sh
    . "$ROBOT_SYSTEM"/shell/complete/gittools.sh

fi



##############################################################################################
#   source setup.sh files
##############################################################################################

# source .sh files
if _bender_check_if_zsh ; then
    setopt shwordsplit
fi
for _pkg in $ROBOT_PACKAGES
do
    _pkg_path="$(echo "$_pkgs_and_paths" | grep "$_pkg " | sed 's/^.* //')"

    # deprecated
    _setup_file="$_pkg_path"/bash/setup.sh

    # source if neccesary.
    if [ -f "$_setup_file" ]; then
    	#echo " - [BENDER DEPRECATED] loading $_setup_file. Please rename the bash/ to shell/."
        . "$_setup_file"
    fi
    
    _setup_file="$_pkg_path"/shell/setup.sh
    if [ -f "$_setup_file" ]; then
        . "$_setup_file"
    fi
done
if _bender_check_if_zsh ; then
    unsetopt shwordsplit
fi
unset _pkgs_and_paths
unset _pkg
unset _pkg_path
unset _setup_file


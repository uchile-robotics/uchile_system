#!/bin/sh

# prevent failure
if ! _uch_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi


# useful function to prevent autocompletion
# usage:
# bash > complete -F "_uchcomplete_NOT_COMPLETE" "target-function"
# zsh  > compctl  -K "_uchcomplete_NOT_COMPLETE" "target-function"
function _uchcomplete_NOT_COMPLETE
{
	if _uch_check_if_bash ; then
    	: # noop
	else
		reply=()
	fi
    return 0
}

#!/bin/sh

# prevent failure
if ! _uchile_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi


# useful function to prevent autocompletion
# usage:
# bash > complete -F "_uchilecomplete_NOT_COMPLETE" "target-function"
# zsh  > compctl  -K "_uchilecomplete_NOT_COMPLETE" "target-function"
function _uchilecomplete_NOT_COMPLETE
{
	if _uchile_check_if_bash ; then
    	: # noop
	else
		reply=()
	fi
    return 0
}


# _uchilecomplete_only_help
# just provides -h and --help options
function _uchilecomplete_only_help
{
    local cur opts prev

    # available options
    opts="-h --help"
    if _uchile_check_if_bash ; then
        # bash - complete
        COMPREPLY=()

        # current word
        cur="${COMP_WORDS[COMP_CWORD]}"
        if [[ $COMP_CWORD == 1 ]] ; then
            COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
            return 0
        fi
    else
        # zsh - compctl
        reply=()
        if [[ ${CURRENT} == 2 ]]; then
            reply=(${=opts})
        fi
    fi
}
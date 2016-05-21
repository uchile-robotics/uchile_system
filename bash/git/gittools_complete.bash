#!/bin/bash

# prevent failure
if [ ! "$CATKIN_SHELL" = "bash" ]; then
    printf "\e[33m[WARNING]: Attempted to run a complete.bash on a sh environment.
    This autocomplete script was designed for bash shells.\e[0m\n\n"
    return 1
fi


_bendercomplete_bender_git ()
{
    local cur opts prev opts_co opts_ls

    # available options
    opts="-h --help"
    opts="status st checkout co ls-files fetch"
    opts_co="master develop"
    opts_ls="modified untracked ignored"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]] ; then
        # autocomplete with any option
        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        return 0
    elif [[ $COMP_CWORD == 2 ]]; then

        prev="${COMP_WORDS[1]}"
        if [ "$prev" = "checkout" ] || [ "$prev" = "co" ]; then
            # autocomplete common branches
            COMPREPLY=( $(compgen -W "${opts_co}" -- "${cur}") )
            return 0

        elif [ "$prev" = "ls-files" ]; then
            # autocomplete
            COMPREPLY=( $(compgen -W "${opts_ls}" -- "${cur}") )
            return 0
        fi
        return 0
    fi
}

complete -F "_bendercomplete_bender_git" "bgit"

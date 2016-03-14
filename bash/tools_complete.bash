#!/bin/bash

# prevent failure
if [ ! "$CATKIN_SHELL" = "bash" ]; then
    printf "\e[33m[WARNING]: Attempted to run complete.bash on a sh environment.
    This autocomplete script was designed for bash shells.\e[0m\n\n"
    return 1
fi


_bendercomplete_bender_find_string()
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} base soft high system graveyard forks install embedded all"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]] ; then
        # autocomplete with any option
        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        return 0
    elif [[ $COMP_CWORD == 2 ]]; then

        # don't autocomplete if previous option was -h or --help
        prev="${COMP_WORDS[1]}"
        if [ "$prev" = "-h" ] || [ "$prev" = "--help" ]; then
            return 0
        fi
        COMPREPLY=( $(compgen -W "\\\"\\\"" -- "${cur}") )
        return 0
    fi
}
complete -F "_bendercomplete_bender_find_string" "bender_find_string"

_bendercomplete_bender_cd()
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} base soft high system graveyard forks install embedded"
    opts="${opts} ${BENDER_PACKAGES}"

    # empty reply
    COMPREPLY=()

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"

    # only complete one option
    if [[ $COMP_CWORD == 1 ]] ; then
        COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        return 0
    fi
}
complete -F "_bendercomplete_bender_cd" "bender_cd" "cdb"
complete -F "_bendercomplete_bender_cd" "bender_ws"


# other tool complete
complete -F "_bendercomplete_NOT_COMPLETE" "bender_printenv"
complete -F "_bendercomplete_NOT_COMPLETE" "bender_refresh_bash"
complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_untracked"
complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_ignored"

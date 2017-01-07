#!/bin/sh

# prevent failure
if ! _bender_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi


if _bender_check_if_bash ; then

    # export functions
    export -f bender_cd
    export -f bender_find_string
    export -f bender_ws

else
    echo " - exporting functions"
fi

_bendercomplete_bender_find_string()
{
    if _bender_check_if_bash ; then
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
    else
        echo " - _bendercomplete_bender_find_string"
    fi
}

_bendercomplete_bender_cd()
{
    if _bender_check_if_bash ; then
        local cur opts prev

        # available options
        opts="-h --help"
        opts="${opts} base soft high system graveyard forks install embedded"
        opts="${opts} ${BENDER_PACKAGES}"
        opts="${opts} ${BENDER_STACKS}"

        # empty reply
        COMPREPLY=()

        # current word
        cur="${COMP_WORDS[COMP_CWORD]}"

        # only complete one option
        if [[ $COMP_CWORD == 1 ]] ; then
            COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
            return 0
        fi
    else
        echo "_bendercomplete_bender_cd"
    fi
}



if _bender_check_if_bash ; then

    complete -F "_bendercomplete_bender_find_string" "bender_find_string"
    complete -F "_bendercomplete_bender_cd" "bender_cd" "cdb"
    complete -F "_bendercomplete_bender_cd" "bender_ws"

    # no completion
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_printenv"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_refresh_bash"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_untracked"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_ignored"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_open_config"

else

    # no completion
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_printenv"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_refresh_bash"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_git_show_untracked"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_git_show_ignored"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_open_config"
fi

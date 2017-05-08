#!/bin/sh

# prevent failure
if ! _bender_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi

## _bendercomplete_bender_find_string
# 1. autocomplete with any option
# 2. don't autocomplete if previous option was -h or --help
function _bendercomplete_bender_find_string
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} base soft high system graveyard forks install embedded all"

    if _bender_check_if_bash ; then
        # bash - complete
        COMPREPLY=()

        # current word
        cur="${COMP_WORDS[COMP_CWORD]}"

        if [[ $COMP_CWORD == 1 ]] ; then
            COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
        elif [[ $COMP_CWORD == 2 ]]; then
            prev="${COMP_WORDS[1]}"
            if [ "$prev" = "-h" ] || [ "$prev" = "--help" ]; then
                return 0
            fi
            COMPREPLY=( $(compgen -W "\\\"\\\"" -- "${cur}") )
        fi
    else
        # zsh - compctl
        reply=()

        if [[ ${CURRENT} == 2 ]]; then
            reply=(${=opts})
        elif [[ ${CURRENT} == 3 ]]; then
            prev="${=${(s: :)words}[2]}"
            if [ "$prev" = "-h" ] || [ "$prev" = "--help" ]; then
                return 0
            fi
            reply=('""')
        fi
    fi
    return 0
}


# _bendercomplete_bender_cd
# single option completion
# 1. workspaces + packages + metapackages
function _bendercomplete_bender_cd
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} base soft high system graveyard forks embedded"
    opts="${opts} $ROBOT_PACKAGES"
    opts="${opts} $ROBOT_STACKS"

    if _bender_check_if_bash ; then
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



if _bender_check_if_bash ; then

    complete -F "_bendercomplete_bender_find_string" "bender_find_string"
    complete -F "_bendercomplete_bender_cd" "bender_cd" "cdb"

    # no completion
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_printenv"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_refresh_shell"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_untracked"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_git_show_ignored"
    complete -F "_bendercomplete_NOT_COMPLETE" "bender_open_config"

else

    compctl -Q -K "_bendercomplete_bender_find_string" "bender_find_string"
    compctl -K "_bendercomplete_bender_cd" "bender_cd"

    # no completion
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_printenv"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_refresh_shell"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_git_show_untracked"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_git_show_ignored"
    compctl -K "_bendercomplete_NOT_COMPLETE" "bender_open_config"
fi

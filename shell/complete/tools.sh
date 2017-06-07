#!/bin/sh

# prevent failure
if ! _uchile_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi

## _uchilecomplete_uchile_find_string
# 1. autocomplete with any option
# 2. don't autocomplete if previous option was -h or --help
function _uchilecomplete_uchile_find_string
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} system forks base soft high misc deps pkgs ros_ws all"

    if _uchile_check_if_bash ; then
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


# _uchilecomplete_uchile_cd
# single option completion
# 1. workspaces + packages + metapackages
function _uchilecomplete_uchile_cd
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} system forks base soft high misc deps pkgs"
    opts="${opts} $UCHILE_PACKAGES"
    opts="${opts} $UCHILE_STACKS"

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

# _uchilecomplete_uchile_make
# single option completion: forks base soft high
function _uchilecomplete_uchile_make
{
    local cur opts prev

    # available options
    opts="-h --help"
    opts="${opts} forks base soft high"

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

if _uchile_check_if_bash ; then

    complete -F "_uchilecomplete_uchile_find_string" "uchile_find_string"
    complete -F "_uchilecomplete_uchile_cd" "uchile_cd" "cdb" "cdu"
    complete -F "_uchilecomplete_only_help" "uchile_fix_links" "uchile_clean_workspace"
    complete -F "_uchilecomplete_uchile_make" "uchile_make"

    # no completion
    complete -F "_uchilecomplete_NOT_COMPLETE" "uchile_printenv"
    complete -F "_uchilecomplete_NOT_COMPLETE" "uchile_refresh_shell"
    complete -F "_uchilecomplete_NOT_COMPLETE" "uchile_git_show_untracked"
    complete -F "_uchilecomplete_NOT_COMPLETE" "uchile_git_show_ignored"
    complete -F "_uchilecomplete_NOT_COMPLETE" "uchile_open_config"

else

    compctl -Q -K "_uchilecomplete_uchile_find_string" "uchile_find_string"
    compctl -K "_uchilecomplete_uchile_cd" "uchile_cd"
    compctl -K "_uchilecomplete_only_help" "uchile_fix_links" "uchile_clean_workspace"
    compctl -K "_uchilecomplete_uchile_make" "uchile_make"

    # no completion
    compctl -K "_uchilecomplete_NOT_COMPLETE" "uchile_printenv"
    compctl -K "_uchilecomplete_NOT_COMPLETE" "uchile_refresh_shell"
    compctl -K "_uchilecomplete_NOT_COMPLETE" "uchile_git_show_untracked"
    compctl -K "_uchilecomplete_NOT_COMPLETE" "uchile_git_show_ignored"
    compctl -K "_uchilecomplete_NOT_COMPLETE" "uchile_open_config"
fi

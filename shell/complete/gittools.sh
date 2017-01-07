#!/bin/sh

# prevent failure
if ! _bender_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi



_bendercomplete_bender_git ()
{
    if _bender_check_if_bash ; then
        local cur opts prev opts_co opts_ls

        # available options
        opts="-h --help"
        opts="${opts} status st checkout co ls-files fetch merge pull"
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
    else
        return 0
    fi
}

if _bender_check_if_bash ; then
    complete -F "_bendercomplete_bender_git" "bgit"
else
    echo "complete _bendercomplete_bender_git"
fi

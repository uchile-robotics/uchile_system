#!/bin/sh

# prevent failure
if ! _uchile_check_if_bash_or_zsh ; then
    printf "\e[33m[WARNING]: Attempted to source this script on an invalid shell env.
    This script was only designed for autocompletion on bash/zsh shells.\e[0m\n\n"
    return 1
fi



# autocompletions:
# 1st: all options
# 2nd: checkout: common local branches
# 2nd: ls-files: ls-files related options
# 2nd: merge: common origin/ branches
_uchilecomplete_uchile_git ()
{
    local cur opts prev opts_co opts_ls opts_mg

    # available options
    opts="-h --help"
    opts="${opts} status st checkout co ls-files fetch merge pull"
    opts_co="master develop"
    opts_ls="modified untracked ignored"
    opts_mg="origin/master origin/develop"


    if _uchile_check_if_bash ; then
        # bash - complete
        COMPREPLY=()

        # current word
        cur="${COMP_WORDS[COMP_CWORD]}"
        if [[ $COMP_CWORD == 1 ]] ; then
            COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )

        elif [[ $COMP_CWORD == 2 ]]; then

            prev="${COMP_WORDS[1]}"
            if [ "$prev" = "checkout" ] || [ "$prev" = "co" ]; then
                COMPREPLY=( $(compgen -W "${opts_co}" -- "${cur}") )

            elif [ "$prev" = "ls-files" ]; then
                COMPREPLY=( $(compgen -W "${opts_ls}" -- "${cur}") )

            # elif [ "$prev" = "merge" ]; then
            #     COMPREPLY=( $(compgen -W "${opts_mg}" -- "${cur}") )
            fi
        fi
    else
        # zsh - compctl
        reply=()

        if [[ ${CURRENT} == 2 ]]; then
            reply=(${=opts})

        elif [[ ${CURRENT} == 3 ]]; then

            prev="${=${(s: :)words}[2]}"
            if [ "$prev" = "checkout" ] || [ "$prev" = "co" ]; then
                reply=(${=opts_co})

            elif [ "$prev" = "ls-files" ]; then
                reply=(${=opts_ls})

            # elif [ "$prev" = "merge" ]; then
            #     reply=(${=opts_mg})
            fi
        fi
    fi
    return 0
}

if _uchile_check_if_bash ; then
    complete -F "_uchilecomplete_uchile_git" "bgit"
else
    compctl -K "_uchilecomplete_uchile_git" "bgit"
fi

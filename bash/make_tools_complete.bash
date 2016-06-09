#!/bin/bash

# prevent failure
if [ ! "$CATKIN_SHELL" = "bash" ]; then
    printf "\e[33m[WARNING]: Attempted to run complete.bash on a sh environment.
    This autocomplete script was designed for bash shells.\e[0m\n\n"
    return 1
fi

# AUN NO ESTAN DISPONIBLES!
return 0

_bendercomplete_bender_make()
{
    local cur prev opts packages

    # available options + bender_ packages
    opts="--eclipse -h --help clean"
    packages=$BENDER_PACKAGES
    opts="${opts} ${packages}"

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

complete -F "_bendercomplete_bender_make" "bender_make" "bender_make_Debug" "bender_make_Release" "bender_make_RelWithDebInfo"

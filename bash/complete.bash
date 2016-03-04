#!/bin/bash

# prevent failure
if [ ! "$CATKIN_SHELL" = "bash" ]; then
    printf "\e[33m[WARNING]: Attempted to run complete.bash on a sh environment.
    This autocomplete script was designed for bash shells.\e[0m\n\n"
    return 1
fi


# useful function to prevent autocompletion
# usage: complete -F "_bendercomplete_NOT_COMPLETE" "my-function1" "my-funtion2" ...
_bendercomplete_NOT_COMPLETE() 
{
    return 0
}


_bendercomplete_str()
{
	local cur

    # current word
    cur="${COMP_WORDS[COMP_CWORD]}"

    # only complete one option
    if [[ $COMP_CWORD == 1 ]] ; then
        COMPREPLY=( $(compgen -W "\\\"\\\"" -- "${cur}") )
        return 0
    fi
}

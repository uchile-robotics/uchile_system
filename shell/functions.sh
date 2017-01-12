#!/bin/sh

_bender_admin_goodbye () 
{
    python "$BENDER_SYSTEM"/shell/src/functions.py "admin_goodbye"

}

_bender_check_if_bash ()
{
    python "$BENDER_SYSTEM"/shell/src/functions.py "shell_is_bash"
}

_bender_check_if_zsh ()
{
    python "$BENDER_SYSTEM"/shell/src/functions.py "shell_is_zsh"
}

_bender_check_if_bash_or_zsh ()
{
    python "$BENDER_SYSTEM"/shell/src/functions.py "shell_is_bash_or_zsh"
}

_bender_check_var_isset ()
{
    python "$BENDER_SYSTEM"/shell/src/functions.py "check_var_isset" "$1"
}

_bender_check_file_ext ()
{
	python "$BENDER_SYSTEM"/shell/src/functions.py "check_file_ext" "$1" "$2"
}

_bender_get_filepath ()
{
	python "$BENDER_SYSTEM"/shell/src/functions.py "get_filepath" "$1"
}

_bender_check_user_confirmation ()
{
    local answer
    printf "Are you sure you want to do this? [y/N]: "
    read answer
    if echo "$answer" | grep -iq "^y" ;then
        return 0
    else
        return 1
    fi
}

_bender_get_active_ip ()
{
    ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | sed 's/\/.*//'
}

_bender_check_installed ()
{
    ## this only works on bash and zsh!!.
    ## 
    if [ -z "$1" ]; then
        echo "_bender_check_installed requires 1 argument: <executable_name>"
        _bender_admin_goodbye
        return 1
    fi

    local name="$1"
    printf " - checking existense of: '%s' ...... " "$name"
    if ! hash "$name" 2>/dev/null; then
        printf " NOT INSTALLED.\n"
        return 1
    else
        printf " OK.\n"
        return 0
    fi
}

_bender_check_installed_and_exit ()
{
    ## this only works on bash and zsh!!.
    ## 
    if [ -z "$1" ]; then
        echo "_bender_check_installed_and_exit requires 1 argument: <executable_name>"
        _bender_admin_goodbye
        exit 1
    fi

    local name="$1"
    if ! _bender_check_installed "$name"; then
        printf "Please Install '%s' before proceeding.\n" "$name"
        exit 1
    fi
    return 0
}

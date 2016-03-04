#!/bin/sh

_bender_check_installed ()
{
    ## this only works on bash!!.
    ## 
    if [ -z "$1" ]; then
        echo "_bender_check_installed requires 1 argument: <executable_name>"
        return 1
    fi

    local name="$1"
    printf "Checking existense of: '%s' ...... " "$name"
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
    ## this only works on bash!!.
    ## 
    if [ -z "$1" ]; then
        echo "_bender_check_installed_and_exit requires 1 argument: <executable_name>"
        exit 1
    fi

    local name="$1"
    if ! _bender_check_installed "$name"; then
        printf "Please Install '%s' before proceeding.\n" "$name"
        exit 1
    fi
    return 0
}

_bender_check_var_isset ()
{
    if [ -z "$1" ]; then
        echo "_bender_check_var_isset requires 1 argument: <var_name>"
        exit 1
    fi

    local varname="$1"
    if [ -z "${!varname+x}" ]; then
        #echo "var '$varname' is unset"
        return 1
    else
        #echo "var is set to ${!varname}"
        return 0
    fi
}

# 
# _bender_check_file_ext <filename> <ext> 
# - returns 0: if <ext> is the file extension of <filename> 
# - returns 1: otherwise
_bender_check_file_ext ()
{
    if [ -z "$1" ]; then
        echo "_bender_check_var_isset requires 2 arguments: <filename> <ext>"
        exit 1
    fi
    if [ -z "$2" ]; then
        echo "_bender_check_var_isset requires 2 arguments: <filename> <ext>"
        exit 1
    fi

    local _filename="$1"
    local _fileext="$2"

    local _basename=${_filename##*/}
    local _name=${_basename%.*}
    local _ext=${_basename##*.}
    local _basepath=${_filename%/*}

    # no extension!
    if [ "$_basename" = "$_ext" ]; then 
        _ext="" 
    fi

    # echo "file: $_jsgf_file"
    # echo "basename: ${_basename}"
    # echo "basepath: ${_basepath}"
    # echo "name: ${_name}"
    # echo "ext : ${_ext}"
    # echo ""

    # check
    if [ "$_fileext" = "$_ext" ]; then
        return 0
    else
        echo "File extension not supported for file: $_basename"
        echo " - found: '.$_ext'."
        echo " - must be: '.$_fileext'."
        return 1
    fi
}


# 
# _bender_get_filepath <filename> 
# - writes the path to stdout.
# - example: filename=/usr/bin/foo path=/usr/bin
# - example: filename=../bin/foo path=../bin
# - example: filename=foo path=.
_bender_get_filepath ()
{
    if [ -z "$1" ]; then
        echo "_bender_get_filepath requires 1 argument: <filename>"
        exit 1
    fi
    local _filename="$1"
    local _basepath=${_filename%/*}

    if [ "$_basepath" = "$_filename" ]; then
        _basepath="."
    fi

    # response
    echo "$_basepath"
}

# 
# _bender_get_file_noextname <filename> 
# - writes the name to stdout.
# - example: filename=/usr/bin/foo   name=foo
# - example: filename=../bin/foo.bar name=foo
# - example: filename=foo            name=foo
# - example: filename=foo.tar.gz     name=foo
_bender_get_file_noextname ()
{
    if [ -z "$1" ]; then
        echo "_bender_get_filebasename requires 1 argument: <filename>"
        exit 1
    fi
    local _filename="$1"
    local _basename=${_filename##*/}
    local _name=${_basename%%.*}

    # response
    echo "$_name"
}

# # tests
# _bender_get_file_noextname "/usr/bin/foo"
# _bender_get_file_noextname "../bin/foo.bar"
# _bender_get_file_noextname "foo"
# _bender_get_file_noextname "foo.tar.gz"

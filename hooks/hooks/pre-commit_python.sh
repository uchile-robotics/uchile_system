#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - PY_FILES
#
#
# Exports:
# - _FAILED

# check command existence
if ! _bender_git_hooks_command_exists "flake8"; then
    return 0
fi

# Flake8 setup
# ------------------------------------
# see $ flake8 --help
#
# code list: https://flake8.readthedocs.org/en/latest/warnings.html
OPTIONS=""

## show statistics
OPTIONS=$OPTIONS" --statistics"

## show source
OPTIONS=$OPTIONS" --show-source"


## Error Codes to ignore
# - C***: McCabe complexity plugin
# - F403: ‘from module import *’ used; unable to detect undefined names
OPTIONS=$OPTIONS" --ignore=C"

OPTIONS=$OPTIONS",E2,E3"
OPTIONS=$OPTIONS",W1,W2,W3"
OPTIONS=$OPTIONS",E126,E127,E128"
#OPTIONS=$OPTIONS",E121,E122,E123,E124,E125,E126,E127,E128,E129,E131,E133"
OPTIONS=$OPTIONS",F401"
OPTIONS=$OPTIONS",F841"
OPTIONS=$OPTIONS",E501"


## Error Codes to include
# i assume the remaining codes are catched by flake8 
#SELECT=$SELECT" --select=F402,F404,F8"

## max-line-length:
# - recommended: 79 (in order to view files side by side)
OPTIONS=$OPTIONS" --max-line-length=120"

_show_py_goodbye() {

    cat <<\EOF

 - (FAILED): Some python files are wrong!.. please fixed before commiting.
 
EOF
}

_py_failed="no"
if [ -z "$PY_FILES" ]; then
    echo " - (OK) there are no staged python files to check :)"
else
    #echo "there are some files to process"
    #echo "files: "$PYTHON_FILES
    #echo "options:"$OPTIONS
    # shellcheck disable=SC2086
    flake8 $OPTIONS $PY_FILES
    # nota: ignorar recomendación SC2086!, al aplicarla, falla el script!
    #.. pues hay un espacio que sobra en un nombre de archivo.

    _rc="$?"
    if [ "$_rc" != "0" ]; then
    	#echo "$_rc"
        _py_failed="yes"
    fi

fi


# failed
if [ "$_py_failed" != "no" ]; then
    _show_py_goodbye
    export _FAILED="yes"
else
    if [ -n "$PY_FILES" ]; then
        echo " - (OK) all staged python files have passed the linter."
    fi  
fi

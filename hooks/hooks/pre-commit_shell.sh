#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - BASH_FILES
# - SH_FILES
#
# Exports:
# - _FAILED

# TODO:
# - seleccionar códigos específicos para el checker
# - comando se ejecuta dos veces :(
#
# setup
# ------------------------------------
# see $ shellcheck --help
#
# code list: https://github.com/koalaman/shellcheck/wiki

# check command existence
if ! _uchile_git_hooks_command_exists "shellcheck"; then
    return 0
fi


_sh_failed="no"

_show_bash_message() {

cat <<\EOF
 
 - (FAILED):"
 Sorry, but bash scripts must have a shebang on the first line.
 Consider adding something like:
 - #!/bin/bash
EOF

}

_show_sh_message() {

cat <<\EOF
 - (FAILED):
 Sorry, but shell scripts must have a shebang on the first line.
 Consider adding something like:
 - #!/bin/sh
 - #!/bin/bash
EOF

}
 
_show_sh_goodbye() {

    cat <<\EOF

 If any, at least you should fix the SC1* codes (the red ones!)
 note: SC2 codes (the green ones) are warnings.

EOF
}

_checkfile() {
    
    _shell="$1"
    _file="$2"
    
    # run to help the user
    shellcheck --shell="$_shell" "$_file"

    # run to check error or warning code
    _found_error=$(shellcheck --shell="$_shell" "$_file" | grep -c -e 'SC1')
    if [ "$_found_error" != "0" ]; then
        _sh_failed="yes"
    fi 
}


## Manage *.sh files
## -------------------------------------------------
# - .sh pueden ser sh o bash
if [ -z "$SH_FILES" ]; then
    echo " - (OK) there are no staged sh files to check :)"
fi
for file in $SH_FILES
do

    _firstline=$(head -n 1 "$file")
    _shebang=$(echo "$_firstline" | grep '^#!/.*sh')
    _bash_bang=$(echo "$_firstline" | grep '^#!/.*/bash')
    _sh_bang=$(echo "$_firstline" | grep '^#!/.*/sh')

    # no shebang
    if [ "$_firstline" = "" ]; then

        echo "In $file:"
        _show_sh_message
        _sh_failed="yes"
        continue
    fi

    # invalid shebang
    if [ "$_shebang" = "" ]; then

        echo "In $file, i found: $_firstline."
        _show_sh_message
        _sh_failed="yes"
        continue
    fi


    # sh bang
    if [ ! -z "$_sh_bang" ]; then

        _checkfile "sh" "$file"
        continue
    fi

    # bash bang
    if [ ! -z "$_bash_bang" ]; then
        
        _checkfile "bash" "$file"
        continue
    fi

    echo "In $file, i found: $_firstline."
    _show_sh_message
    _sh_failed="yes"

done


## Manage *.bash files
## -------------------------------------------------
# - .bash sólo pueden ser bash
if [ -z "$BASH_FILES" ]; then
    echo " - (OK) there are no staged bash files to check :)"
fi
for file in $BASH_FILES
do

    _firstline=$(head -n 1 "$file")
    _shebang=$(echo "$_firstline" | grep '^#!/.*/bash')

    # no shebang
    if [ "$_firstline" = "" ]; then

        echo "In $file:"
        _show_bash_message
        _sh_failed="yes"
        continue
    fi

    # invalid shebang
    if [ "$_shebang" = "" ]; then

        echo "In $file, i found: $_firstline."
        _show_bash_message
        _sh_failed="yes"
        continue
    fi

    _checkfile "bash" "$file"
    
done


# failed
if [ "$_sh_failed" != "no" ]; then
    _show_sh_goodbye
    export _FAILED="yes"
else
    if [ -n "$BASH_FILES" ] || [ -n "$SH_FILES" ] ; then
        echo " - (OK) all staged shell (bash/sh) files have passed the linter. ... hurra for you! :)"
    fi
fi

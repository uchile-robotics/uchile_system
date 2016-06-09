#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - FILES
#
#
# Exports:
# - _FAILED

# check command existence
if ! _bender_git_hooks_command_exists "awk"; then
    return 0
fi

_max_size="400" # kB
_max_size_B=$(awk "BEGIN {printf \"%d\",${_max_size}*1024}")
_max_size_MB=$(awk "BEGIN {printf \"%.2f\",${_max_size}/1024}")

_show_size_goodbye() {

    cat <<\EOF
 - (FAILED): BLAME ON YOU!
 Please, consider another alocation method for that files.
 e.g: The Team's Dropbox

EOF
}

_max_found_B="0" # B
_size_failed="no"
for _file in $FILES
do
    #echo "working on size file: "$file
    _size=$(stat --format="%s" "$_file")

    if [ "$_size" -gt "$_max_found_B" ]; then
        _max_found_B="$_size"
    fi

    if [ "$_size" -gt "$_max_size_B" ]; then
        _human_size=$(ls -lh "$_file"  | awk '{ print $5 }')
        echo " "
        echo "[FAILED]: OVERSIZED FILE!"
        echo " - File $_file is too big for git!: $_human_size (max: $_max_size_MB MB)"
        echo " "
        _size_failed="yes"
    fi
done

# failed
if [ "$_size_failed" != "no" ]; then
    _show_size_goodbye
    export _FAILED="yes"
else
    _max_found_KB=$(awk "BEGIN {printf \"%.2f\",${_max_found_B}/1024}")
    echo " - (OK) all staged files are not oversized. (max: $_max_size_MB MB, max found: $_max_found_KB kB)"
fi


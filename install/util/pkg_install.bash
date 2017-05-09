#!/bin/bash

# allows re sourcing robot scripts on installers
unset UCH_FRAMEWORK_TWICE_LOAD_CHECK

# re source bash
if [ -z "$UCH_WS" ]; then

	# compatibility with v1.9
	if [ ! -z "$BENDER_WS" ]; then
		export UCH_WS="$BENDER_WS"
	else
		echo "uch framework UCH_WS env var not found"
		echo "The installation cannot proceed. Bye"
		exit 0
	fi
fi

if [ -d "$UCH_WS"/bender_system ]; then
	# compatibility with v1.9
	source "$UCH_WS"/bender_system/setup.bash
else
	source "$UCH_WS"/system/setup.bash
fi

#!/bin/bash

# allows re sourcing robot scripts on installers
unset ROBOT_FRAMEWORK_TWICE_LOAD_CHECK

# re source bash
if [ -z "$ROBOT_WS" ]; then

	# compatibility with v1.9
	if [ ! -z "$BENDER_WS" ]; then
		export ROBOT_WS="$BENDER_WS"
	else
		echo "uch framework ROBOT_WS env var not found"
		echo "The installation cannot proceed. Bye"
		exit 0
	fi
fi

if [ -d "$ROBOT_WS"/bender_system ]; then
	# compatibility with v1.9
	source "$ROBOT_WS"/bender_system/setup.bash
else
	source "$ROBOT_WS"/system/setup.bash
fi

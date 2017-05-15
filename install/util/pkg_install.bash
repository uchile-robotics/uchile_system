#!/bin/bash

# allows re sourcing robot scripts on installers
unset UCHILE_FRAMEWORK_TWICE_LOAD_CHECK

# re source bash
if [ -z "$UCHILE_WS" ]; then

	# compatibility with v1.9
	if [ ! -z "$BENDER_WS" ]; then
		export UCHILE_WS="$BENDER_WS"
	else
		echo "uchile framework UCHILE_WS env var not found"
		echo "The installation cannot proceed. Bye"
		exit 0
	fi
fi

if [ -d "$UCHILE_WS"/bender_system ]; then
	# compatibility with v1.9
	source "$UCHILE_WS"/bender_system/setup.bash
else
	source "$UCHILE_WS"/system/setup.bash
fi

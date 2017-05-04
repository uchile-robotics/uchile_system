#!/bin/bash

# allows re sourcing the bender scripts on installers
unset BENDER_FRAMEWORK_TWICE_LOAD_CHECK

# re source bash
if [ -z "$BENDER_WS" ]; then
	echo "bender_framework BENDER_WS env var not found"
	echo "The installation cannot proceed. Bye"
	exit 0
fi
source "$BENDER_WS"/bender_system/setup.bash
#!/bin/sh

## checks for command existence
# usage: _bender_git_hooks_command_exists "my-command"
# returns 0: if "my-command" command exists
# retunrs 1: otherwise
_bender_git_hooks_command_exists ()
{
	if command -v "$1" >/dev/null 2>&1; then
		return 0
	fi

	printf " - (UNKNOWN): Command '%s' is not installed.\n" "$1"
	return 1
}

_bender_git_hooks_pymodule_exists ()
{
	if python -c "import $1" >/dev/null 2>&1; then
		return 0
	fi

	printf " - (UNKNOWN): Python module '%s' is not installed.\n" "$1"
	return 1
}


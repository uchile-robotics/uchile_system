#!/usr/bin/python
#
# this file is only intended for launching "meld" in a proper way
# when running $(git diff ...)
#
# it requires the following parameter to be set on the gitconfig file:
# - diff.external ~/.gitconfig_meld_launcher.py
#
# where this file is copied into the home directory or is symlinked there
#
#
# matias.pavez.b
#

import sys
import os

os.system('meld "%s" "%s"' % (sys.argv[2], sys.argv[5]))
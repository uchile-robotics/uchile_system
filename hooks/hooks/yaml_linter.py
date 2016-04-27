#!/usr/bin/python
#
# by mpavez at matias.pavez.b@gmail.com 
#

import sys
import yaml

if not len(sys.argv) == 2:
    print "usage: python yaml_linter.py <file>"
    sys.exit(1)


the_file = str(sys.argv[1])
#print 'File: ', the_file

try:
    a_yaml = yaml.load(file(the_file, 'r'))
except yaml.YAMLError, exc:
    print "[YAML SYNTAX ERROR]"
    print "Error found while parsing file:", the_file
    if hasattr(exc, 'problem_mark'):
        mark = exc.problem_mark
        print " - found at line %s, column %s." % (mark.line+1, mark.column+1)
    print " - description: (init)"
    print " --- "
    print exc
    print " --- "
    print " - description: (end)"
    print " "
    print " "
    sys.exit(1)

sys.exit(0)

# obtained from: https://github.com/pre-commit/pre-commit-hooks
from __future__ import print_function

import argparse
import sys
from termcolor import cprint

CONFLICT_PATTERNS = [
    '<<<<<<<',
    '>>>>>>>'
]
WARNING_MSG = '\n ----> found merge conflict string "{0}" on file:\n {1}, line {2}.\n'


def detect_merge_conflict(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('filenames', nargs='*')
    args = parser.parse_args(argv)

    retcode = 0
    for filename in args.filenames:
        with open(filename, 'r') as inputfile:
            for i, line in enumerate(inputfile):
                for pattern in CONFLICT_PATTERNS:
                    if line.startswith(pattern):
                        cprint(WARNING_MSG.format(
                            pattern.decode(), filename, i + 1,
                        ), 'red')
                        retcode = 1

    return retcode

if __name__ == '__main__':
    sys.exit(detect_merge_conflict())

#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - against
#
#
# Exports:
# 


weird_chars=$(git diff --cached --name-only --diff-filter=ACMR -z "$against" | 
	LC_ALL=C tr -d '[:alnum:][/._\-]\0')
# echo "weird: '$weird_chars'"

count=${#weird_chars}
# echo "count: $count"


if [ "$count" != 0 ]
then
	cat <<\EOF
 WTF!...
 Please mind your filenames!

 For the sake of everyone, avoid weird characters.
 These are the 'nice' chars for filenames: [a-zA-Z/._-]
 note: SPACE IS NOT A NICE CHAR!
 note: '(', ')', '[' and ']' ARE NOT NICE CHARS!

EOF
echo " I found $count invalid chars on staged filenames: '$weird_chars'"
cat <<\EOF

 kindly
 git admin.-

 Have some feedback?, please contact us: TODO :p

EOF
	exit 1
else
  echo " - (OK) all submitted files have valid filenames."
fi

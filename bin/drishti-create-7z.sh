#!/bin/bash -e
#
# See: https://gist.github.com/ruslo/817fb379cf150f9c7dbc6514b8ca00c8
# Copyright (C) 2016, Ruslan baratov

# Warning! Commit all the changes you need before running the script!
#
# Rule of thumb:
#
# > git status
## nothing
#
# > git branch -vv
## check that you're on the right branch
#
# > /path/to/create-7z.sh # run this script

set -x

BRANCHNAME="`git rev-parse --abbrev-ref HEAD`"

echo "Packing branch: '${BRANCHNAME}'"

rm -rf _packing ${BRANCHNAME}.tar ${BRANCHNAME}.7z

mkdir _packing

git archive ${BRANCHNAME} > ${BRANCHNAME}.tar

tar -x -f ${BRANCHNAME}.tar -C _packing

(cd _packing && 7z a -t7z -m0=lzma -mx=9 -mfb=128 -md=64m -ms=on ../${BRANCHNAME}.7z .)

ls -la ${BRANCHNAME}.7z

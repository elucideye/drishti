#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=libcxx-hid-fpic

EXTRA_ARGS=""
if [ $# -ge 1 ] && [ $1 -gt 0 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --clear "
fi

if [ $# -ge 2 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --home ${2} "
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--strip "
    "--jobs 8 "
    "--reconfig "
    "${EXTRA_ARGS} "
)

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}


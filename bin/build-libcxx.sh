#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=libcxx-hid

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "--config Release "
    "--pack ${CPACK_TYPE} "
    "--strip "
    "--jobs 8 "
    "${EXTRA_ARGS} "
)

eval build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 

#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=android-ndk-r10e-api-16-x86-hid

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "BUILD_MIN_SIZE=${BUILD_MIN_SIZE} "
    "--config Release "
    "--pack ${CPACK_TYPE} "
    "--strip "
    "--jobs 8 "
    "${EXTRA_ARGS} "
)

eval build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 




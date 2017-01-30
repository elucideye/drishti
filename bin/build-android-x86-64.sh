#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=android-ndk-r10e-api-21-x86-64-hid-sections

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "DRISHTI_BUILD_MIN_SIZE=${DRISHTI_BUILD_MIN_SIZE} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
    "--strip "
    "${EXTRA_ARGS} "
)

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?"

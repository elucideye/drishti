#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=android-ndk-r10e-api-21-x86-64-hid-sections

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
    "--strip "
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?"

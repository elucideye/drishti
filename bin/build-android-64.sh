#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=android-ndk-r10e-api-21-arm64-v8a-gcc-49-hid-sections

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "ANDROID_COMPILER_VERSION=\"4.9\" "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
    "--strip "
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 

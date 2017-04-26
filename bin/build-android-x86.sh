#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=android-ndk-r10e-api-16-x86-hid-sections

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--strip "
    "--jobs 8 "
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 

#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=osx-10-11-sanitize-address
TOOLCHAIN=xcode-sections
DRISHTI_BUILD_SHARED_SDK=ON
DRISHTI_BUILD_ASAN_TEST=OFF
DRISHTI_BUILD_QT=OFF

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=${DRISHTI_CONFIGURATION} "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
    "--install "
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

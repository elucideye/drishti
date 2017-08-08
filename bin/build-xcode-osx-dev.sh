#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

TOOLCHAIN=xcode-hid-sections

DRISHTI_BUILD_LOCAL=(
    "DRISHTI_BUILD_QT=OFF"
    "DRISHTI_COTIRE=OFF"
    "DRISHTI_USE_BEAST=ON"
)

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=Release "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_BUILD_LOCAL[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "        
    "--jobs 8 "
    "--install "
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

TOOLCHAIN=xcode-hid-sections

DRISHTI_BUILD_C_INTERFACE=ON
DRISHTI_BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON
DRISHTI_COTIRE=OFF

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=Release "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "DRISHTI_COTIRE=${DRISHTI_COTIRE} "
    "DRISHTI_BUILD_C_INTERFACE=${DRISHTI_BUILD_C_INTERFACE} "
    "${DRISHTI_POLLY_ARGS[*]} "        
    "--jobs 8 "
    "--install "
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

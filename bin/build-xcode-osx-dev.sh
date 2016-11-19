#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

TOOLCHAIN=xcode-hid-sections

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--reconfig "
fi

DRISHTI_BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON
DRISHTI_BUILD_TESTS=ON
DRISHTI_COTIRE=OFF

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=Release "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "DRISHTI_BUILD_TESTS=${DRISHTI_BUILD_TESTS} "
    "DRISHTI_COTIRE=${DRISHTI_COTIRE} "
    "${DRISHTI_POLLY_ARGS[*]} "        
    "--jobs 8 "
    "--open "
    "--install "
    "--test " 
    "${EXTRA_ARGS}" #  "--nobuild"
)


build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}



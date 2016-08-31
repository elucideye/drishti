#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

TOOLCHAIN=xcode

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    #EXTRA_ARGS="--reconfig"
    EXTRA_ARGS="--clear"
fi

BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=Release "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_OSX_DEPLOYMENT_TARGET=10.10 "
    "CMAKE_OSX_DEPLOYMENT_TARGET=10.10 "
    "CMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT} "
    "BUILD_QT=${BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "--config Release "
    "--jobs 8 "
    "--open "
    "--install "
    "${EXTRA_ARGS}"
)

build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}



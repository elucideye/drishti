#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=xcode

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    #EXTRA_ARGS="--reconfig"
    EXTRA_ARGS="--clear"
fi

DRISHTI_BUILD_QT=OFF
DRISHTI_BUILD_OGLES_GPGPU=OFF
DRISHTI_USE_XML_ARCHIVES=ON

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "HUNTER_CONFIGURATION_TYPES=Release "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_OSX_DEPLOYMENT_TARGET=10.10 "
    "CMAKE_OSX_DEPLOYMENT_TARGET=10.10 "
    "CMAKE_OSX_SYSROOT=${CMAKE_OSX_SYSROOT} "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "DRISHTI_BUILD_MIN_SIZE=${DRISHTI_BUILD_MIN_SIZE} "
    "DRISHTI_USE_XML_ARCHIVES=${DRISHTI_USE_XML_ARCHIVES} "
    "--config Release "
    "--jobs 8 "
    "--open "
    "--install "
    "${EXTRA_ARGS} " #--nobuild"
)

build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}



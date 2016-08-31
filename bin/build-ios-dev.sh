#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

TOOLCHAIN=ios-9-2-arm64
#TOOLCHAIN=ios-9-2-hid

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${DRISHTISDK_IOS_IDENTITY}" ]; then
    echo 2>&1 "Must have DRISHTISDK_IOS_IDENTITY set"
fi

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear --reconfig"
fi

BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET=8.0 "
    "BUILD_QT=${BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "--config Release "
    "--framework-device "
    "--install "
    "--jobs 8 "
    "--open "
    "--plist \"${DRISHTISDK}/cmake/framework/Info.plist\" "
    "--identity \"${DRISHTISDK_IOS_IDENTITY}\" "    
    "${EXTRA_ARGS} "
)

eval build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

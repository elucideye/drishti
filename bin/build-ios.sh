#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${DRISHTISDK_IOS_IDENTITY}" ]; then
    echo 2>&1 "Must have DRISHTISDK_IOS_IDENTITY set"
fi

TOOLCHAIN=ios-9-2-hid

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--reconfig --clear"
fi

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET=8.0 "
    "--config Release "
    "--framework-device "
    "--ios-multiarch "
    "--install "
    "--jobs 8 "
    "--open "
    "--plist \"${DRISHTISDK}/cmake/framework/Info.plist\" "
    "--identity \"${DRISHTISDK_IOS_IDENTITY}\" "
    "${EXTRA_ARGS} "
)

eval build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}






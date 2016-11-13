#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=ios-10-0-dep-8-0-hid

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

rename_tab drishti $TOOLCHAIN

COMMANDS=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET=8.0 "
    "--framework-device "
    "--install "
    "--jobs 8 "
    "--open "
    "--plist \"${DRISHTISDK}/cmake/framework/Info.plist\" "
    "--identity "${IOS_IDENTITY}" "
    "${EXTRA_ARGS} "
)

build.py --toolchain ${TOOLCHAIN} ${COMMANDS[*]}






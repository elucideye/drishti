#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

TOOLCHAIN=ios-10-1-arm64-dep-8-0-hid-sections

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${DRISHTISDK_IOS_IDENTITY}" ]; then
    echo 2>&1 "Must have DRISHTISDK_IOS_IDENTITY set"
fi

DRISHTI_BUILD_LOCAL=(
    DRISHTI_BUILD_QT=ON
    DRISHTI_COTIRE=OFF
    DRISHTI_OPENGL_ES3=OFF    
)

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_BUILD_LOCAL[*]} "
    "CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET=9.0 "        
    "${DRISHTI_POLLY_ARGS[*]} "
    "--framework-device "
    "--framework-lib libdrishti.*"
    "--install "
    "--jobs 8 "
    "--plist \"${DRISHTISDK}/cmake/framework/Info.plist\" "
    "--identity \"${DRISHTISDK_IOS_IDENTITY}\" "    
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

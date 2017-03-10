#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=ios-10-1-dep-8-0-libcxx-hid-sections

rename_tab drishti $TOOLCHAIN

COMMANDS=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET=8.0 "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--framework-device "
    "--ios-multiarch --ios-combined --archive drishti"
    "--install "
    "--jobs 8 "
    "--plist ${DRISHTISDK}/cmake/framework/Info.plist "
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMANDS[*]} --identity "${DRISHTISDK_IOS_IDENTITY}"

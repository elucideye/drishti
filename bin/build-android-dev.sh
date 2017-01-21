#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

DRISHTI_BUILD_C_INTERFACE=OFF
DRISHTI_BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON
DRISHTI_BUILD_TESTS=ON
DRISHTI_COTIRE=OFF

# Qt friendly (no -hid-sections)
TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon  # Qt tests

# Test 32-bit
#TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections

# test 64-bit
#TOOLCHAIN=android-ndk-r10e-api-21-arm64-v8a-gcc-49-hid-sections

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    #EXTRA_ARGS="--reconfig " #--clear"
    EXTRA_ARGS="--reconfig --clear"
fi

[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_SDK_ROOT
[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_HOME
echo $ANDROID_SDK_ROOT

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose "
    "--fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "    
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "DRISHTI_BUILD_TESTS=${DRISHTI_BUILD_TESTS} "
    "DRISHTI_COTIRE=${DRISHTI_COTIRE} "
    "DRISHTI_BUILD_C_INTERFACE=${DRISHTI_BUILD_C_INTERFACE} "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--jobs 8 "
    "--test "
    "--install "    
    "${EXTRA_ARGS} " 
)

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

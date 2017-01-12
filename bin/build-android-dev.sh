#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

#TOOLCHAIN=android-ndk-r10e-api-16-armeabi-v7a-neon-clang-35-hid
#TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon

# default
TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon
#TOOLCHAIN=android-ndk-r10e-api-16-armeabi-v7a-neon-clang-35-hid
#TOOLCHAIN=android-ndk-r10e-api-16-armeabi-v7a-neon-clang-35-hid-sections-asan
#TOOLCHAIN=android-ndk-r11c-api-21-armeabi-v7a-neon-clang-35

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--reconfig " #--clear"
fi

DRISHTI_BUILD_C_INTERFACE=OFF
DRISHTI_BUILD_QT=OFF
DRISHTI_BUILD_OGLES_GPGPU=ON
DRISHTI_BUILD_TESTS=ON
DRISHTI_COTIRE=OFF

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
    "${EXTRA_ARGS} " #    "--install "
)

eval build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

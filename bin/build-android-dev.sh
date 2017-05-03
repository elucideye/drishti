#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

DRISHTI_BUILD_LOCAL=(
    DRISHTI_BUILD_QT=OFF
    DRISHTI_COTIRE=OFF
)

TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections

[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_SDK_ROOT
[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_HOME
echo $ANDROID_SDK_ROOT

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose "
    "--fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_BUILD_LOCAL[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
    "--strip "    
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

grep -o -e ".*launch:" _builds/${TOOLCHAIN}-${DRISHTI_BUILD_CONFIG}/Makefile

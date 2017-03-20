#!/bin/bash

. ${DRISHTISDK}/bin/build-common-defs-devel.sh

### customize ###
HUNTER_CONFIGURATION_TYPES=MinSizeRel
DRISHTI_BUILD_CONFIG=MinSizeRel

DRISHTI_SERIALIZE_WITH_BOOST=OFF
DRISHTI_SERIALIZE_WITH_CEREAL=ON
DRISHTI_SERIALIZE_WITH_CVMATIO=OFF

DRISHTI_BUILD_C_INTERFACE=ON
DRISHTI_BUILD_QT=ON
DRISHTI_BUILD_OGLES_GPGPU=ON
DRISHTI_BUILD_TESTS=ON
DRISHTI_COTIRE=OFF
### customize ###

set_polly_args

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
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_COTIRE=${DRISHTI_COTIRE} "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--jobs 8 "
    "--strip "    
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

grep -o -e ".*launch:" _builds/${TOOLCHAIN}-${DRISHTI_BUILD_CONFIG}/Makefile

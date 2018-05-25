#!/bin/bash -e
#
# Simple build drishti SDK build script to support both CI builds
# and matching host development.

# Sample input parameteters:
# TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections-lto
# CONFIG=MinSizeRel
# INSTALL=--strip

if [ ${#} != 3 ]; then
    echo 2>&1 "usage: drishti_build.sh TOOLCHAIN CONFIG INSTALL"
    exit 1
fi

TOOLCHAIN="${1}"
CONFIG="${2}"
INSTALL="${3}"

if [[ ${TRAVIS} == "true" ]]; then
    GAUZE_ANDROID_USE_EMULATOR=YES # remote test w/ emulator
else
    GAUZE_ANDROID_USE_EMULATOR=NO # support local host testing on a real device
fi

# Note: '--ios-{multiarch,combined}' do nothing for non-iOS builds
ARGS=(
    --toolchain "${TOOLCHAIN}"
    --config "${CONFIG}"
    --verbose
    --ios-multiarch --ios-combined
    --archive drishti
    --jobs 2
    ${TEST}
    "${INSTALL}"
    --fwd
    DRISHTI_USE_DRISHTI_UPLOAD=YES
    DRISHTI_BUILD_SHARED_SDK=${BUILD_SHARED}
    DRISHTI_BUILD_TESTS=YES
    DRISHTI_BUILD_EXAMPLES=YES
    DRISHTI_COPY_3RDPARTY_LICENSES=ON
    DRISHTI_HAS_GPU=${GPU}
    GAUZE_ANDROID_USE_EMULATOR=${GAUZE_ANDROID_USE_EMULATOR}
    GAUZE_ANDROID_EMULATOR_GPU=swiftshader
    GAUZE_ANDROID_EMULATOR_PARTITION_SIZE=40
    HUNTER_CONFIGURATION_TYPES="${CONFIG}"
    HUNTER_SUPPRESS_LIST_OF_FILES=ON
)

polly.py ${ARGS[@]} --reconfig

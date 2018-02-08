#!/bin/bash
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

# Note: '--ios-{multiarch,combined}' do nothing for non-iOS builds
ARGS=(
    --toolchain "${TOOLCHAIN}"
    --config "${CONFIG}"
    --verbose
    --ios-multiarch --ios-combined
    --fwd
    DRISHTI_BUILD_SHARED_SDK=YES
    DRISHTI_BUILD_TESTS=YES
    DRISHTI_BUILD_EXAMPLES=YES
    DRISHTI_COPY_3RDPARTY_LICENSES=ON
    GAUZE_ANDROID_USE_EMULATOR=YES
    HUNTER_CONFIGURATION_TYPES="${CONFIG}"
    HUNTER_SUPPRESS_LIST_OF_FILES=ON
    --archive drishti
    --jobs 2
    --test      
    "${INSTALL}"
)

polly.py ${ARGS[@]} --reconfig

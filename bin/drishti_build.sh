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

if [ -z "${CI}" ]; then
    # For non-CI builds with arbitrary toolchains we don't require cache usage:
    HUNTER_USE_CACHE_SERVERS=YES
    HUNTER_DISABLE_BUILDS=NO
else
    # For CI builds with strict timeouts we must use a server side cache:
    HUNTER_USE_CACHE_SERVERS=ONLY
    HUNTER_DISABLE_BUILDS=YES
fi

# Note: '--ios-{multiarch,combined}' do nothing for non-iOS builds
ARGS=(
    --toolchain "${TOOLCHAIN}"
    --config "${CONFIG}"
    --verbose
    --ios-multiarch --ios-combined
    --fwd
    DRISHTI_BUILD_TESTS=YES
    DRISHTI_BUILD_EXAMPLES=YES
    DRISHTI_COPY_3RDPARTY_LICENSES=ON
    GAUZE_ANDROID_USE_EMULATOR=YES
    HUNTER_USE_CACHE_SERVERS="${HUNTER_USE_CACHE_SERVERS}"
    HUNTER_DISABLE_BUILDS="${HUNTER_DISABLE_BUILDS}"
    HUNTER_CONFIGURATION_TYPES="${CONFIG}"
    HUNTER_SUPPRESS_LIST_OF_FILES=ON
    --archive drishti
    --jobs 2
    --test      
    "${INSTALL}"
)

polly.py ${ARGS[@]} --reconfig

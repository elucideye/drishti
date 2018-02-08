#!/bin/bash

CONFIG=MinSizeRel
TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections-lto
INSTALL=--strip

ARGS=(
    --toolchain ${TOOLCHAIN}
    --config ${CONFIG}
    --verbose
    --ios-multiarch --ios-combined
    --fwd
    DRISHTI_BUILD_REGRESSION_SIMD=NO
    DRISHTI_BUILD_REGRESSION_FIXED_POINT=NO
    DRISHTI_BUILD_TESTS=YES
    DRISHTI_BUILD_EXAMPLES=YES
    DRISHTI_COPY_3RDPARTY_LICENSES=ON
    GAUZE_ANDROID_USE_EMULATOR=YES
    HUNTER_CONFIGURATION_TYPES=${CONFIG}
    HUNTER_SUPPRESS_LIST_OF_FILES=ON
    --archive drishti
    --jobs 2
    --test      
    ${INSTALL}
)

polly.py ${ARGS[@]} --reconfig


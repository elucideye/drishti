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

# On CI/Travis + Linux the cpu emulation is having problems (OK on mac hosts)
# C++ exception with description "EGLContextImpl::EGLContextImpl() : eglCreatePbufferSurface(): 
# Assertion '(eglGetError() == EGL_SUCCESS)' failed in file 'lib/aglet/EGLContext.cpp' line 59"
# It seems the cpu based gpu emulator associated with GAUZE_ANDROID_EMULATOR_GPU=off either isn't 
# working or isn't compatible with this eglCreatePbufferSurface() stuff, however, the build is fine
# and the tests run on a real Android via USB connection in local host testing
# See: https://developer.android.com/studio/run/emulator-acceleration#command-gpu

if [[ $(uname -a) =~ .*Linux.* ]]; then
    GAUZE_ANDROID_EMULATOR_GPU=off # this should work
    is_linux=1
else
    GAUZE_ANDROID_EMULATOR_GPU=host
fi

# skip emulator CI tests for linux builds
if [[ ! -z ${TRAVIS+x} ]] && [[ ! -z ${is_linux+x} ]]; then
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
    --test      
    "${INSTALL}"
    --fwd
    DRISHTI_USE_DRISHTI_UPLOAD=YES
    DRISHTI_BUILD_SHARED_SDK=YES
    DRISHTI_BUILD_TESTS=YES
    DRISHTI_BUILD_EXAMPLES=YES
    DRISHTI_COPY_3RDPARTY_LICENSES=ON
    GAUZE_ANDROID_USE_EMULATOR=${GAUZE_ANDROID_USE_EMULATOR}
    GAUZE_ANDROID_EMULATOR_GPU=${GAUZE_ANDROID_EMULATOR_GPU}
    HUNTER_CONFIGURATION_TYPES="${CONFIG}"
    HUNTER_SUPPRESS_LIST_OF_FILES=ON
)

polly.py ${ARGS[@]} --reconfig

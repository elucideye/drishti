#!/bin/bash
#
# Integration tests (install + packaging) 
#

#!/bin/bash

. ${DRISHTISDK}/bin/version.sh

PATH="/usr/bin:${PATH}"

# Replace with input args
OS_NAME=$1
TOOLCHAIN=$2

### Top level folder for custom multi-platform packaging
RELEASE_DIR="${DRISHTISDK}/releases/drishti_${DRISHTI_VERSION}/drishti/${OS_NAME}"

DRISHTI_DO_TEST=1

if [[ "${TOOLCHAIN}" =~ ^ios-* ]] ; then
    echo "IS_IOS___"
    drishti_DIR="${RELEASE_DIR}/Frameworks"
    OpenCV_DIR="${RELEASE_DIR}/3rdparty/Frameworks"
else

    ### Drishti SDK
    drishti_DIR="${RELEASE_DIR}/share/drishti"

    ### IO libs 
    ZLIB_DIR="${RELEASE_DIR}/3rdparty/lib/cmake/ZLIB"
    PNG_DIR="${RELEASE_DIR}/3rdparty/lib/cmake/PNG"

    ### OpenCV and static dependencies (for IO)    
    if [[ "${TOOLCHAIN}" =~ ^android-* ]] ; then
        OpenCV_DIR="${RELEASE_DIR}/3rdparty/sdk/native/jni"

        if [[ "${TOOLCHAIN}" == *x86* ]]; then
            echo "SKIPPING TEST..."
            DRISHTI_DO_TEST=0
        fi
        
    else
        OpenCV_DIR="${RELEASE_DIR}/3rdparty/share/OpenCV"
    fi
fi

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${RELEASE_DIR}" ]; then
    echo 2>&1 "Can't find release directory for TOOLCHAIN=${TOOLCHAIN} at expect location $RELEASE_DIR"
fi

ARGS=(
    "--output=/tmp"
    "--clear "        
    "--verbose "
    "--config Release "
    "--toolchain ${TOOLCHAIN} "
    "--fwd ${ARGS[@]} "
    "drishti_DIR=${drishti_DIR}"
    "OpenCV_DIR=${OpenCV_DIR}"
    "DRISHTI_BUILD_INTEGRATION_TESTS=ON"
)

# IO libs required by opencv except in opencv.framework
if [ -n "${ZLIB_DIR}" ]
then
    ARGS+=( "ZLIB_DIR=${ZLIB_DIR}" )
fi

if [ -n "${PNG_DIR}" ]
then
    ARGS+=( "PNG_DIR=${PNG_DIR}" );
fi

if [ ${DRISHTI_DO_TEST} -gt 0 ]
then
    ARGS+=( --test )
fi

# Run tests except on x86 Android platforms:

INTEGRATION_DIR=${DRISHTISDK}/src/examples/integration

# NOTE: We need to make sure our integration test doesn't overwrite our top _install
polly.py ${ARGS[@]}

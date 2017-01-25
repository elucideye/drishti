#!/bin/bash
#
# Integration tests (install + packaging) 
#

#!/bin/bash

. ${DRISHTISDK}/bin/version.sh

# Replace with input args
OS_NAME=$1
TOOLCHAIN=$2

### Top level folder for custom multi-platform packaging
RELEASE_DIR="${DRISHTISDK}/releases/drishti_${DRISHTI_VERSION}/drishti/${OS_NAME}"

if [[ "${TOOLCHAIN}" =~ ^ios-* ]] ; then
    drishti_DIR="${RELEASE_DIR}/Frameworks"
    OpenCV_DIR="${RELEASE_DIR}/3rdparty/Frameworks"
else
    ### Drishti SDK
    drishti_DIR="${RELEASE_DIR}/share/drishti"

    ### OpenCV and static dependencies (for IO)
    OpenCV_DIR="${RELEASE_DIR}/3rdparty/share/OpenCV"
    ZLIB_DIR="${RELEASE_DIR}/3rdparty/lib/cmake/ZLIB"
    PNG_DIR="${RELEASE_DIR}/3rdparty/lib/cmake/PNG"
fi

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${RELEASE_DIR}" ]; then
    echo 2>&1 "Can't find release directory for TOOLCHAIN=${TOOLCHAIN} at expect location $RELEASE_DIR"
fi

ARGS=(\
    "--clear "
    "--toolchain ${TOOLCHAIN} "
    "--fwd ${ARGS[@]} "
    "drishti_DIR=${drishti_DIR}"
    "OpenCV_DIR=${OpenCV_DIR}"
    "ZLIB_DIR=${ZLIB_DIR}"
    "PNG_DIR=${PNG_DIR}"
)

polly.py --home ${DRISHTISDK}/src/examples/integration ${ARGS[@]}

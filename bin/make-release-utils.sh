#!/bin/bash -x

# opencv_framework:
#
# Here is where we build the opencv2.framework (iOS (or OS X) only)
#
# Samples:
# TOOLCHAIN=ios-8-4
# EXPORT_DIR=${EVDIR}/prebuilts/drishti/ios

. ${DRISHTISDK}/bin/build-common-release.sh

function opencv_framework
{
    TOOLCHAIN=$1
    EXPORT_DIR=$2
    HUNTER_INSTALL_DIR=$3

    DST_DIR=_install/${TOOLCHAIN}/3rdparty/opencv2
    DST_INC_DIR=${DST_DIR}/include
    DST_LIB_DIR=${DST_DIR}/lib
    DST_FWK_DIR=${DST_DIR}/fwk

    mkdir -p  ${DST_LIB_DIR}
    mkdir -p  ${DST_INC_DIR}
    mkdir -p  ${DST_FWK_DIR}
    cp -r ${HUNTER_INSTALL_DIR}/include/opencv2 ${DST_INC_DIR}

    OPENCV_SRC=${HUNTER_INSTALL_DIR}/lib/libopencv_world.a
    OPENCV_DST=${DST_LIB_DIR}/libopencv2.a
    cp ${OPENCV_SRC} ${OPENCV_DST}

## HEREDOC 
python3 <<EOF
import sys
sys.path.append('${POLLY_ROOT}/bin') 
sys.path.append('${POLLY_ROOT}/bin/detail/')
import detail.logging
logging=detail.logging.Logging('/tmp', 1, 10, 100)
import create_framework
plist='${DRISHTISDK}/cmake/framework/Info.plist'
identity='${DRISHTISDK_IOS_IDENTITY}'
create_framework.run('${DST_DIR}', '${DST_FWK_DIR}', '8.0', '${POLLY_ROOT}', 0, logging, plist, identity)
EOF

    FRAMEWORK_DIR=${EXPORT_DIR}/3rdparty/Frameworks
    mkdir -p ${FRAMEWORK_DIR}
    cp -r ${DST_FWK_DIR}/opencv2.framework ${FRAMEWORK_DIR}
}

DRISHTI_LIBRARY_EXCLUSION_LIST=(
    "/boost"
    "GPUImage"
    "android-sdk"
    "boost_"
    "cascades"
    "cereal"
    "cvmatio"
    "cxxopts"
	"dlib"    
    "eigen3"
    "flatbuffers"
    "gmock"
    "gtest"
    "half"
    "nlohmann"
    "spdlog"
    "thread-pool"
    "thread_pool"
    "licenses" # use _install <package>_LICENSE instead
)

function export_dependencies 
{
    TOOLCHAIN=$1
    EXPORT_DIR=$2
    HUNTER_INSTALL_DIR=$3

    ### Install opencv and common dependencies only (minimize export of utility libs)
	EXCLUSION_PATTERN="$(echo ${DRISHTI_LIBRARY_EXCLUSION_LIST[@]} | tr ' ' '|')"

    # Here we exclude some binary packages, but we need to retain the licenses:
    [ -f /tmp/staging.tar ] && rm /tmp/staging.tar

    (
        cd ${HUNTER_INSTALL_DIR}
        find . | grep -i -E "${EXCLUSION_PATTERN}"  > /tmp/Exclude
        tar cfX /tmp/staging.tar /tmp/Exclude .
    )
    (cd ${EXPORT_DIR} && mkdir -p 3rdparty && cd 3rdparty && tar zxf /tmp/staging.tar)
}

function make_release
{
    TOOLCHAIN=$1
    EXPORT_DIR=$2
    IS_RELEASE=$3
    INSTALL_DIR=$4

    HUNTER_BASE=${HOME}/.hunter/_Base

    echo $@

    ### Compute Hunter CONIG_ID and TOOLCHAIN_ID
    BUILD_DIR=${INSTALL_DIR}/_builds/${TOOLCHAIN}
    if [ $IS_RELEASE -gt 0 ]; then
		BUILD_DIR="${BUILD_DIR}-Release"
    fi

    HUNTER_INSTALL_DIR=$(awk '{print $1}' ${BUILD_DIR}/_3rdparty/Hunter/install-root-dir)

    echo "TOOLCHAIN=$TOOLCHAIN"
    echo "BUILD_DIR=$BUILD_DIR"
    echo "HUNTER_INSTALL_DIR=${HUNTER_INSTALL_DIR}"
    
    LIB_DIR=${HUNTER_INSTALL_DIR}/lib

    ### Prerequisites
    [ ! -d ${INSTALL_DIR}/_install/${TOOLCHAIN} ] && >&2 echo "Requested build does not exist for toolchain ${TOOLCHAIN}.  Export failed" && exit 1 
    [ ! -d ${HUNTER_INSTALL_DIR} ] && >&2 echo "Hunter build tree does not exist for toolchain ${TOOLCHAIN}.  Export failed" && exit 1 
    
    ### Install the drishti libs
    # Clean the existing deps and start anew
    [ -d ${EXPORT_DIR} ] && rm -rf ${EXPORT_DIR}
    mkdir -p ${EXPORT_DIR} 

    [ -f /tmp/drishti.tar ] && rm /tmp/drishti.tar
    (
        cd ${INSTALL_DIR}/_install/${TOOLCHAIN}

        # Example:
        # .
        # ├── 3rdparty
        # │   └── licenses
        # ├── bin
        # ├── include
        # │   └── drishti
        # ├── lib
        # └── share
        # └── drishti
        #echo -e "bin\n3rdparty" > /tmp/Exclude 

        # Exclude installed 3rdparty static libs from bundle, but keep license subdirectory
        # Currently xgboost, various boost libs reside in the top level folder
        # This occures in drishti/src/lib/drishti/CMakeLists.txt.
        find 3rdparty -depth 1 -name "*.a" > /tmp/Exclude
        tar cfX /tmp/drishti.tar /tmp/Exclude .
        cd ${EXPORT_DIR} && tar xf /tmp/drishti.tar
    )

    #### Install the dependencies (Export 3rd party libs for reuse)
    export_dependencies ${TOOLCHAIN} ${EXPORT_DIR} ${HUNTER_INSTALL_DIR}

    # If TOOLCHAIN matches ios-*
    if [[ "${TOOLCHAIN}" =~ ^ios-* ]] ; then
		DRISHTI_FRAMEWORK=_framework/${TOOLCHAIN}/drishti.framework
		FRAMEWORK_DIR=${EXPORT_DIR}/Frameworks
		mkdir -p ${FRAMEWORK_DIR}

		echo "TOOLCHAIN ${TOOLCHAIN}"
		echo "EXPORT_DIR ${EXPORT_DIR}"
		echo "HUNTER_INSTALL_DIR ${HUNTER_INSTALL_DIR}"

		[ -d ${DRISHTI_FRAMEWORK} ] && cp -r ${DRISHTI_FRAMEWORK} ${FRAMEWORK_DIR}
		opencv_framework ${TOOLCHAIN} ${EXPORT_DIR} ${HUNTER_INSTALL_DIR}
    fi
}

## Prerequisites
function check_prerequisites
{
    [ ! -d ${DRISHTISDK} ] && >&2 echo "Must specify valid POLLY_ROOT" && exit 1
    [ ! -d ${POLLY_ROOT} ] && >&2 echo "Must specify valid DRISHTISDK" && exit 1
}

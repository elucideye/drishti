#!/bin/bash -x

. ${DRISHTISDK}/bin/make-release-utils.sh
. ${DRISHTISDK}/bin/version.sh
. ${DRISHTISDK}/bin/toolchains.sh
. ${DRISHTISDK}/bin/license.sh

set -e

echo $DRISHTI_VERSION

START=$1
END=$2

RELEASE_NAME=drishti_${DRISHTI_VERSION} ### TODO centralize
RELEASE_DIR=${DRISHTISDK}/releases
DRISHTI_DIR=${RELEASE_DIR}/${RELEASE_NAME}/drishti

##################################################
## Create a license patch for all architectures ##
##################################################
LICENSE_DIR=${DRISHTI_DIR}/licenses
LICENSE_PATCH=${DRISHTI_DIR}/licenses.tgz
mkdir -p ${LICENSE_DIR}
drishti_license_path ${LICENSE_DIR}
(cd ${DRISHTI_DIR} && tar zcvf licenses.tgz licenses)

for((i=${START}; i<${END}; i++))
do
    EXPORT_DIR=${DRISHTI_DIR}/${NAMES[i]}
    mkdir -p ${EXPORT_DIR}
    make_release ${TOOLCHAINS[i]} ${EXPORT_DIR} ${RELEASES[i]} ${INSTALL_DIR[i]}
    
    ### Bin
    TEST_APP=${INSTALL_DIR}/_install/${TOOLCHAINS[i]}/bin/test-drishti-segment
    if [ -f $TEST_APP ];
    then
        OUT_BIN_DIR=${EXPORT_DIR}/bin
        mkdir -p ${OUT_BIN_DIR} && cp ${TEST_APP} ${OUT_BIN_DIR}
    fi

    ## License patch:
    echo ${NAMES[i]}    
    (cd ${EXPORT_DIR}/3rdparty && echo ${PWD} && tar zxvf ${LICENSE_PATCH})

    echo ${NAMES[i]}
    
done
rm ${LICENSE_PATCH} # clean up 

### Model data
# TODO

drishti_eye_model=$(find ${DRISHTISDK} -name "drishti_eye_full_npd_eix.pba.z" | head -1)
if [ -f "$drishti_eye_model" ]; then
    mkdir -p ${DRISHTI_DIR}/assets
    cp "${drishti_eye_model}" ${DRISHTI_DIR}/assets/drishti_eye_eix.pba.z
fi

# Copy the main LICENSE file:
cp ${DRISHTISDK}/LICENSE ${DRISHTI_DIR}

# Create *.tgz builds w/ and w/o the symbol files:
(
    cd $RELEASE_DIR
    tar zcvf ${RELEASE_NAME}.debug.tgz ${RELEASE_NAME}    
    tar zcvf ${RELEASE_NAME}.tgz --exclude=*.dSYM --exclude=*.build-id ${RELEASE_NAME}
)


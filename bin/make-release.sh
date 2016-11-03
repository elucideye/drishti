#!/bin/bash

. ${DRISHTISDK}/bin/make-release-utils.sh
. ${DRISHTISDK}/bin/version.sh
. ${DRISHTISDK}/bin/toolchains.sh

set -e

echo $DRISHTI_VERSION

DRISHTI_DIR=${DRISHTISDK}/releases/drishti_${DRISHTI_VERSION}/drishti

RELEASE_NAME=drishti_${DRISHTI_VERSION} ### TODO centralize

for((i=0; i<${#NAMES[@]}; i++))
do
    EXPORT_DIR=${DRISHTI_DIR}/${NAMES[i]}
    mkdir -p ${EXPORT_DIR}
    make_release ${TOOLCHAINS[i]} ${EXPORT_DIR} ${RELEASES[i]} ${INSTALL_DIR[i]}

    ### Bin
    TEST_APP=${INSTALL_DIR}/_install/${TOOLCHAINS[i]}/bin/test-drishti-segment
    if [ -f $TEST_APP ];
    then
        OUT_BIN_DIR=${DRISHTISDK}/releases/${RELEASE_NAME}/drishti/${NAMES[i]}/bin/
        mkdir -p ${OUT_BIN_DIR} && cp ${TEST_APP} ${OUT_BIN_DIR}
    fi
done

### Model data
# TODO
mkdir -p ${DRISHTI_DIR}/assets
cp  ${DRISHTISDK}/assets/drishti_eye_full_npd_eix.pba.z ${DRISHTI_DIR}/assets/drishti_eye_eix.pba.z

### License


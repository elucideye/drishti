#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=libcxx-hid-sections
#TOOLCHAIN=libcxx-hid

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--reconfig"
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "DRISHTI_BUILD_MIN_SIZE=ON "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--archive drishti"
    "--jobs 8 " # install/strip target missing in CMake 3.7.1
    "--install "
    "${EXTRA_ARGS} "
)

if [ ${DRISHTI_DO_TESTING} -gt 0 ];
then
    COMMAND+=( "--test" )
fi

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 

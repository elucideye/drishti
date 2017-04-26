#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=libcxx-hid-sections
#TOOLCHAIN=libcxx-hid

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--archive drishti"
    "--jobs 8 " # install/strip target missing in CMake 3.7.1
    "--install "
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?" 

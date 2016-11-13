#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

TOOLCHAIN=libcxx-omp

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

DRISHTI_BUILD_QT=OFF
DRISHTI_BUILD_OGLES_GPGPU=ON

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd DRISHTI_BUILD_DOCUMENTATION=ON"
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_EXPORT_COMPILE_COMMANDS=ON "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--strip "
    "--jobs 8 "
    "--reconfig "
    "${EXTRA_ARGS} "
    "--test "
)

build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?"

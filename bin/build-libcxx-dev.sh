#!/bin/bash

. ${DRISHTISDK}/bin/build-dev.sh

TOOLCHAIN=libcxx-omp

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

BUILD_QT=OFF
BUILD_OGLES_GPGPU=ON

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd DRISHTI_BUILD_DOCUMENTATION=ON"
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_EXPORT_COMPILE_COMMANDS=ON "
    "BUILD_QT=${BUILD_QT} "
    "BUILD_OGLES_GPGPU=${BUILD_OGLES_GPGPU} "
    "--config Release "
    "--strip "
    "--jobs 8 "
    "--reconfig "
    "${EXTRA_ARGS} "
    "--test "
)

build.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?"

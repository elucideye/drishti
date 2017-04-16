#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

TOOLCHAIN=libcxx-hid-sections

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

DRISHTI_BUILD_QT=OFF

rename_tab drishti ${TOOLCHAIN}

COMMAND=(
    "--verbose --fwd DRISHTI_BUILD_DOCUMENTATION=ON"
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "CMAKE_EXPORT_COMPILE_COMMANDS=ON "
    "DRISHTI_BUILD_QT=${DRISHTI_BUILD_QT} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--strip "
    "--jobs 8 "
    "--reconfig "
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

echo "Finished with status $?"

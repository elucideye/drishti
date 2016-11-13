#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

# polly/sanitize-address.cmake
# polly/sanitize-leak.cmake
# polly/sanitize-memory.cmake
# polly/sanitize-thread.cmake

TOOLCHAIN=sanitize-address
#TOOLCHAIN=sanitize-memory

EXTRA_ARGS=""
if [ $# -ge 1 ]; then
    EXTRA_ARGS="--clear"
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

COMMANDS=(
    "--verbose "
    "--fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "BUILD_UTILITIES=ON "
    "${DRISHTI_POLLY_ARGS[*]} "    
    "--install "
    "--jobs 8 "
    "${EXTRA_ARGS} "
)

build.py --toolchain ${TOOLCHAIN} ${COMMANDS[*]}

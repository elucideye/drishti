#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

# polly/sanitize-address.cmake
# polly/sanitize-leak.cmake
# polly/sanitize-memory.cmake
# polly/sanitize-thread.cmake

TOOLCHAIN=sanitize-address
#TOOLCHAIN=sanitize-memory

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
)

COMMAND+=( $(add_polly_commands "$@") )

polly.py --toolchain ${TOOLCHAIN} ${COMMANDS[*]}

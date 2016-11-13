#!/bin/bash

. ${DRISHTISDK}/bin/build-common.sh

TOOLCHAIN=gcc-hid-fpic

EXTRA_ARGS=""
if [ $# -ge 1 ] && [ $1 -gt 0 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --clear "
fi

if [ $# -ge 2 ]; then
    EXTRA_ARGS="${EXTRA_ARGS} --home ${2} "
fi

[ ! -d ${DRISHTISDK}/_logs ] && mkdir -p ${DRISHTISDK}/_logs

rename_tab drishti ${TOOLCHAIN}

CMAKE_SHARED_LDFLAGS="-Wl,-Bsymbolic-functions -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now"
EXTRA_LD_FLAGS=(
    CMAKE_MODULE_LINKER_FLAGS=\"${CMAKE_SHARED_LDFLAGS}"\" 
    CMAKE_SHARED_LINKER_FLAGS=\"${CMAKE_SHARED_LDFLAGS}"\"
)

COMMAND=(
    "--toolchain ${TOOLCHAIN} "
    "--verbose "
    "--fwd "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${EXTRA_LD_FLAGS[*]} "
    "--strip "
    "--test "
    "--jobs 8 "
    "--reconfig "
    "${EXTRA_ARGS} "
)

eval build.py ${COMMAND[*]}

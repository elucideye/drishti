#!/bin/bash

. ${DRISHTISDK}/bin/build-common-release.sh

TOOLCHAIN=gcc-hid-fpic

# if [ $# -ge 2 ]; then
#     EXTRA_ARGS="${EXTRA_ARGS} --home ${2} "
# fi

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
    "${DRISHTI_POLLY_ARGS[*]} "
    "--strip "
    "--jobs 8 "
    "--reconfig "
)

COMMAND+=( $(add_polly_commands "$@") )

eval polly.py ${COMMAND[*]}

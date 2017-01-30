#!/bin/bash

. ${DRISHTISDK}/bin/toolchains.sh
. $(which env_parallel.bash)

# Add explicit check for POLLY_IOS_DEVELOPMENT_TEAM before script launch,
# although polly will complain in the build command if this is missing too.
if [ -z "${POLLY_IOS_DEVELOPMENT_TEAM}" ]; then
    echo 2>&1 "Must have POLLY_IOS_DEVELOPMENT_TEAM set"
fi

set -e

RECONFIG=1
START=0
END=${#NAMES[@]}
LAST=$((END - 1))

function build_lib
{
    echo "running ${1} : ${TOOLCHAINS[$1]}"
    ./bin/${BUILDER[$1]} ${RECONFIG} &> _builds/${TOOLCHAINS[$1]}_build.txt
}
export -f build_lib

function build_integration
{
    echo "testing ${1} : ${TOOLCHAINS[$1]}"
    ./bin/build-integration-test.sh ${NAMES[$1]} ${TOOLCHAINS[$1]} &> _builds/${TOOLCHAIN[$1]}_test.txt     
}
export -f build_integration

#
# Run a first pass build + install per platform
#

env_parallel build_lib ::: $(seq ${START} ${LAST})
echo "BUILD exit: $?"

#
# Create a multi-arch release tree
#

./bin/make-release.sh ${START} ${END}

#
# Build integration tests from the release tree
#

env_parallel build_integration ::: $(seq ${START} ${LAST})
echo "TEST exit: $?"

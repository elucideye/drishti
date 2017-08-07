#!/bin/bash

. ${DRISHTISDK}/bin/toolchains.sh

# Add explicit check for POLLY_IOS_DEVELOPMENT_TEAM before script launch,
# although polly will complain in the build command if this is missing too.
if [ -z "${POLLY_IOS_DEVELOPMENT_TEAM}" ]; then
    echo 2>&1 "Must have POLLY_IOS_DEVELOPMENT_TEAM set"
    exit 1
fi

set -e

RECONFIG=1

START=0
END=${#NAMES[@]}

#
# Run a first pass build + install per platform
#

for((i=${START}; i<${END}; i++))
do
    echo ${NAMES[i]}
    ./bin/${BUILDER[i]} ${RECONFIG}
done

#
# Create a multi-arch release tree
#

./bin/make-release.sh ${START} ${END}

#
# Build integration tests from the release tree
#

for((i=${START}; i<${END}; i++))
do
    echo ${NAMES[i]}    
    ./bin/build-integration-test.sh ${NAMES[i]} ${TOOLCHAINS[i]}
done

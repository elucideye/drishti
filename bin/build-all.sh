#!/bin/bash

. ${DRISHTISDK}/bin/toolchains.sh

# Add explicit check for POLLY_IOS_DEVELOPMENT_TEAM before script launch,
# although polly will complain in the build command if this is missing too.
if [ -z "${POLLY_IOS_DEVELOPMENT_TEAM}" ]; then
    echo 2>&1 "Must have POLLY_IOS_DEVELOPMENT_TEAM set"
fi

set -e

RECONFIG=1
for((i=0; i<${#NAMES[@]}; i++))
do
    echo ${NAMES[i]}
    ./bin/${BUILDER[i]} ${RECONFIG}
    ./bin/build-integration-test.sh ${NAMES[i]} ${TOOLCHAINS[i]}
done

./bin/make-release.sh

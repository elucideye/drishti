#!/bin/bash

. ${DRISHTISDK}/bin/toolchains.sh

# Add explicit check for POLLY_IOS_DEVELOPMENT_TEAM before script launch,
# although polly will complain in the build command if this is missing too.
if [ -z "${POLLY_IOS_DEVELOPMENT_TEAM}" ]; then
    echo 2>&1 "Must have POLLY_IOS_DEVELOPMENT_TEAM set"
fi

set -e

RECONFIG=0
for((i=0; i<${#NAMES[@]}; i++))
do
	./bin/${BUILDER[i]} ${RECONFIG}
done

./bin/make-release.sh

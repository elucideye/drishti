#!/bin/bash

. ${DRISHTISDK}/bin/toolchains.sh

set -e

RECONFIG=0
for((i=0; i<${#NAMES[@]}; i++))
do
	./bin/${BUILDER[i]} ${RECONFIG}
done

./bin/make-release.sh
./bin/make-installer.sh

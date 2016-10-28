#!/bin/bash

# http://cgold.readthedocs.io/en/latest/first-step/native-build-tool/xcode.html#several-custom-xcode-versions
# export DEVELOPER_DIR=/Applications/develop/ide/xcode/7.2.1/Xcode.app/Contents/Developer
export DEVELOPER_DIR=/Applications/develop/ide/xcode/8.0/Xcode.app/Contents/Developer
xcodebuild -version

. ${DRISHTISDK}/bin/toolchains.sh

set -e

RECONFIG=0
for((i=0; i<${#NAMES[@]}; i++))
do
	./bin/${BUILDER[i]} ${RECONFIG}
done

./bin/make-release.sh

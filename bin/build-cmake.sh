#!/bin/bash
#
# Note:
# brew uninstall --force cmake

URL=https://cmake.org/files/v3.7/cmake-3.7.0-rc2.tar.gz
FILENAME=${URL##*/}
DIRNAME=${FILENAME%.tar.gz}
(cd /tmp && wget ${URL} && tar zxf ${FILENAME} && cd ${DIRNAME} && ./bootstrap && make && make install)


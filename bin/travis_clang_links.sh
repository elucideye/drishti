#!/bin/bash
set -e
link_dir=latest-clang-symlinks
if [[ $TRAVIS_OS_NAME == 'linux' ]]; then 
    mkdir -p ${link_dir}
    for exe in clang clang++ ; do
        ln -s /usr/bin/${exe}-5.0 ${link_dir}/${exe}
    done
    export PATH="${PWD}/${link_dir}:${PATH}"
fi

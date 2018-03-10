#!/bin/bash
#
# Source: https://github.com/ruslo/hunter/blob/master/.travis.yml
#
# Download the latest polly release, install and use install-ci-dependencies.py
# to install a recent CMake that is compatible with Hunter (has CURL, etc).
# This script supports CI builds but can also be used to configure a functional
# Hunter build environment on a host machine for standard development.

if [ -z "${CI}" ]; then

    # Usage: travis_retry <flaky_command>
    # https://github.com/t-richards/dotfiles/blob/master/.bash-functions
    function travis_retry {
        local result=0
        local count=1
        while [ $count -le 3 ]; do
            [ $result -ne 0 ] && {
                echo -e "\n\033[33;1mThe command \"$@\" failed. Retrying, $count of 3.\033[0m\n" >&2
            }
            "$@"
            result=$?
            [ $result -eq 0 ] && break
            count=$(($count + 1))
            sleep 1
        done
        
        [ $count -eq 3 ] && {
            echo "\n\033[33;1mThe command \"$@\" failed 3 times.\033[0m\n" >&2
        }
        return $result
    }
fi

# Add '--quiet' to avoid leaking the token to logs
git submodule update --init --recursive --quiet

# Info about OS
uname -a

# Install Python 3
# >> Error: python 2.7.14 is already installed \n To upgrade to 3.6.4_3, run `brew upgrade python`
# if [[ "$(uname)" == "Darwin" ]] && [ ! $(which python3) ]; then travis_retry brew install python3; fi
if [[ "$(uname)" == "Darwin" ]] && [ ! $(which python3) ]; then travis_retry brew upgrade python; fi

# Install Python package 'requests'
# 'easy_install3' is not installed by 'brew install python3' on OS X 10.9 Maverick
if [[ "$(uname)" == "Darwin" ]]; then pip3 install requests; fi
if [[ "$(uname)" == "Linux" ]]; then travis_retry pip3 install --user requests; fi

# Install latest Polly toolchains and scripts
wget https://github.com/ruslo/polly/archive/master.zip
unzip -o master.zip
POLLY_ROOT="${PWD}/polly-master"
export PATH="${POLLY_ROOT}/bin:${PATH}"

# Install dependencies (CMake, Android NDK)
install-ci-dependencies.py

# Tune locations
export PATH="${PWD}/_ci/cmake/bin:${PATH}"

# Installed if toolchain is Android (otherwise directory doesn't exist)
export ANDROID_NDK_r10e="${PWD}/_ci/android-ndk-r10e"

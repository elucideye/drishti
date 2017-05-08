#!/bin/bash

set -e

[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_SDK_ROOT
[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_HOME
echo $ANDROID_SDK_ROOT

. ${DRISHTISDK}/bin/drishti-password.sh

PATH="/usr/bin:${PATH}" # system python first

# Function to be called in each build script after customizing arguments:
function set_polly_args
{
    # DRISHTI_BUILD_ARGS : see build-common-defs-*.sh
    
    DRISHTI_BUILD_HIDE=(
        "CMAKE_VISIBILITY_INLINES_HIDDEN=ON"
        "CMAKE_CXX_VISIBILITY_PRESET=hidden"
        "CMAKE_XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN=YES"
        "CMAKE_XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN=YES"
    )

    DRISHTI_POLLY_ARGS=(
        "--config=${DRISHTI_BUILD_CONFIG}"
    )
}

function add_polly_commands
{
    # http://stackoverflow.com/a/14203146
    OPTIND=1

    do_launch=0
    extra=()
    while getopts "crtonl" opt; do
        case "$opt" in
            h|\?) 2>&1 echo "Unknown param"; exit 1 ;;
            t) extra+=("--test") ;;
            r) extra+=("--reconfig") ;;
            c) extra+=("--clear") ;;
            o) extra+=("--open") ;;
            n) extra+=("--nobuild") ;;
            l) do_launch=1 ;;
        esac
    done
    shift $((OPTIND-1))
    [ "$1" = "--" ] && shift

    if [ ${do_launch} -gt 0 ];
    then
        extra+=("--launch") # add launch last
    fi
    
    echo ${extra[@]}
}

function rename_tab
{
  echo -ne "\033]0;$1:$2\007"
}

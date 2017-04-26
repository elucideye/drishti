#!/bin/bash

# Function to be called in each build script after customizing arguments:
function set_polly_args
{
    DRISHTI_BUILD_ARGS=(
        "HUNTER_CONFIGURATION_TYPES=${HUNTER_CONFIGURATION_TYPES} "                           
        "DRISHTI_BUILD_EXAMPLES=${DRISHTI_BUILD_EXAMPLES} "
        "DRISHTI_BUILD_TESTS=${DRISHTI_BUILD_TESTS} "
        "DRISHTI_BUILD_FACE=${DRISHTI_BUILD_FACE} "
        "DRISHTI_BUILD_ACF=${DRISHTI_BUILD_ACF} "
        "DRISHTI_BUILD_HCI=${DRISHTI_BUILD_HCI} "
        "DRISHTI_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU} "
        "DRISHTI_BUILD_REGRESSION_FIXED_POINT=${DRISHTI_BUILD_REGRESSION_FIXED_POINT} "
        "DRISHTI_BUILD_REGRESSION_SIMD=${DRISHTI_BUILD_REGRESSION_SIMD} "
        "DRISHTI_SERIALIZE_WITH_BOOST=${DRISHTI_SERIALIZE_WITH_BOOST} "
        "DRISHTI_SERIALIZE_WITH_CEREAL=${DRISHTI_SERIALIZE_WITH_CEREAL} "
        "DRISHTI_SERIALIZE_WITH_CVMATIO=${DRISHTI_SERIALIZE_WITH_CVMATIO} "
        "DRISHTI_USE_TEXT_ARCHIVES=${DRISHTI_USE_TEXT_ARCHIVES} "
        "DRISHTI_DISABLE_DSYM=${DRISHTI_DISABLE_DSYM} "
        "DRISHTI_BUILD_C_INTERFACE=${DRISHTI_BUILD_C_INTERFACE} "
        "DRISHTI_BUILD_EOS=${DRISHTI_BUILD_EOS}"        
        "DRISHTI_BUILD_DEST=${DRISHTI_BUILD_DEST}"
        "DRISHTI_BUILD_CV_ML=${DRISHTI_BUILD_CV_ML}"
        "DRISHTI_BUILD_SHARED_SDK=${DRISHTI_BUILD_SHARED_SDK}"
        "DRISHTI_BUILD_MERGED_SDK=${DRISHTI_BUILD_MERGED_SDK}"
        "DRISHTI_BUILD_MIN_SIZE=${DRISHTI_BUILD_MIN_SIZE}"
    )

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

    extra=()
    while getopts "crton" opt; do
        case "$opt" in
            h|\?) 2>&1 echo "Unknown param"; exit 1 ;;
            t) extra+=("--test") ;;
            r) extra+=("--reconfig") ;;
            c) extra+=("--clear") ;;
            o) extra+=("--open") ;;
            n) extra+=("--nobuild") ;;
        esac
    done
    shift $((OPTIND-1))
    [ "$1" = "--" ] && shift
    echo ${extra[@]}
}



function rename_tab
{
  echo -ne "\033]0;$1:$2\007"
}

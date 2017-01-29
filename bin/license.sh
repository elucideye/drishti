#!/bin/bash
#
# Copy license files to post install tree
#
# REQUIREMENT: wget

license_dir=$1

ruslo_packages=(
    "hunter:LICENSE"
    "polly:LICENSE"
    "sugar:LICENSE"
)

hunter_packages=(
    "xgboost:LICENSE"
    "eigen:COPYING.MPL2"
    "ogles_gpgpu:LICENSE"
    "thread-pool-cpp:LICENSE"
)

local_packages=(
    "boost_pba:${DRISHTISDK}/src/3rdparty/pba/LICENSE_1_0.txt"
    "cxxopts:${DRISHTISDK}/src/3rdparty/cxxopts/LICENSE"    
    "NEONvsSSE:${DRISHTISDK}/src/3rdparty/simd/LICENSE"
)

function drishti_license_path
{
    top_license_dir=$1

    for((j=0; j<${#hunter_packages[@]}; j++))
    do
        entry=${hunter_packages[j]}
        package=${entry%:*}
        license=${entry#*:}

        echo ${entry} " " ${package} " " ${license}
        
        license_url=https://github.com/hunter-packages/${package}/raw/hunter/${license}
        license_dir=${top_license_dir}/${package}
        wget -O ${license_dir}_LICENSE ${license_url}
    done

    for((j=0; j<${#ruslo_packages[@]}; j++))
    do
        entry=${ruslo_packages[j]}
        package=${entry%:*}
        license=${entry#*:}
        
        license_url=https://github.com/ruslo/${package}/raw/master/${license}
        license_dir=${top_license_dir}/${package}
        wget -O ${license_dir}_LICENSE ${license_url}
    done

    for((j=0; j<${#local_packages[@]}; j++))
    do
        entry=${local_packages[j]}
        package=${entry%:*}
        license=${entry#*:}

        license_dir=${top_license_dir}/${package}
        cp ${license} ${license_dir}_LICENSE
    done
}

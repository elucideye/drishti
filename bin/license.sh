#!/bin/bash

function drishti_license_path
{
    license_dir=$1
    
    ## Eigen
    EIGEN_LICENSE_URL=https://github.com/hunter-packages/eigen/raw/master/COPYING.MPL2
    EIGEN_DIR=${license_dir}/Eigen
    mkdir -p ${EIGEN_DIR}
    wget -O ${EIGEN_DIR}/${EIGEN_LICENSE_URL##*/} ${EIGEN_LICENSE_URL}
    
    ## Sugar
    SUGAR_LICENSE_URL=https://github.com/ruslo/sugar/raw/master/LICENSE
    SUGAR_DIR=${license_dir}/Sugar
    mkdir -p ${SUGAR_DIR}
    wget -O ${SUGAR_DIR}/${SUGAR_LICENSE_URL##*/} ${SUGAR_LICENSE_URL}
    
    ## GTest
    GTEST_LICENSE_URL=https://github.com/hunter-packages/gtest/raw/hunter/LICENSE
    GTEST_DIR=${license_dir}/GTest
    mkdir -p ${GTEST_DIR}
    wget -O ${GTEST_DIR}/${GTEST_LICENSE_URL##*/} ${GTEST_LICENSE_URL}
}

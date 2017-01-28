#!/bin/bash

. ${DRISHTISDK}/bin/version.sh

# Provide a release summary:

echo ${DRISHTI_VERSION}

function add_tab
{
    sed 's|^|     |'
}

(
    cd ${DRISHTISDK}/releases/drishti_${DRISHTI_VERSION}/drishti
    find . -name "*.dylib" -or -name "*.so" | while read lib_name
    do
        file ${lib_name}
        
        du -sh ${lib_name} | add_tab

        if [[ "${lib_name}" == *osx* ]] || [[ "${lib_name}" == *ios* ]]; then
            dwarfdump ${lib_name} | add_tab
        fi
    done

    find . -name "*.framework" | grep -v opencv2 | while read framework_path
    do
        framework_name=${framework_path##*/}
        lib_name=${framework_name%.*}
        file ${framework_path}/${lib_name}

        du -sh ${framework_path}/${lib_name} | add_tab
        dwarfdump ${framework_path}/${lib_name} | add_tab
        echo ${lib_name}
    done
)

#!/bin/bash

# Print files with a missing license

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

# Find all internal files, making sure to exlude 3rdparty subprojects
function find_drishti_source()
{
    find ${DRISHTISDK}/src -not \( -path ${DRISHTISDK}/src/3rdparty -prune \) -name "*.h" -or -name "*.cpp" -or -name "*.hpp"
}

find_drishti_source | while read name
do
    grep -e "@author" $name > /dev/null 2>&1
    [ $? -gt 0 ] && echo $name
done



#!/bin/bash

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

# Find all internal files, making sure to exlude 3rdparty subprojects
function find_drishti_source()
{
    find ${DRISHTISDK}/src/ -not \( -path ${DRISHTISDK}/src/3rdparty -prune \) -name "*~" -or -name "*.orig"
}

find_drishti_source | while read name
do
    echo $name
    mv ${name} /tmp/
done

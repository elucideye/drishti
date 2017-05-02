#!/bin/bash

# Note: When using clang-format the ${DRISHTISDK}/.clang-format file
# seems to be ignored if the root directory to this script is not
# specified relative to the working directory:
#
# Suggested usage:
#
# cd ${DRISHTISDK}
# ./bin/drishti-format.sh src/lib
# ./bin/drishti-format.sh src/app
# ./bin/drishti-format.sh src/test

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

# Find all internal files, making sure to exlude 3rdparty subprojects
function find_drishti_source()
{
    NAMES=(-name "*.h" -or -name "*.cpp" -or -name "*.hpp" -or -name "*.m" -or -name "*.mm")
    find $1 -not \( -path ${DRISHTISDK}/src/3rdparty -prune \) ${NAMES[@]}
}

#input_dir=${DRISHTISDK}/src
input_dir=$1

echo ${input_dir}

find_drishti_source ${input_dir} | grep -v "src/3rdparty" | while read name
do
    echo $name
    #astyle --style=allman --add-brackets --indent-switches $name
    clang-format -i -style=file ${name}
done



#!/bin/bash

function find_drishti_source()
{
    find ${DRISHTISDK}/src -not \( -path ${DRISHTISDK}/src/3rdparty -prune \) -name "*.h" -or -name "*.cpp" -or -name "*.hpp"
}

# skip 3rdparty dir
find_drishti_source | xargs etags --append

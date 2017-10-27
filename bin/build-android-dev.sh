#!/bin/bash

. ${DRISHTISDK}/bin/build-common-devel.sh

DRISHTI_BUILD_LOCAL=(
    DRISHTI_BUILD_QT=OFF
    DRISHTI_COTIRE=OFF
    DRISHTI_OPENGL_ES3=ON
)

TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections

[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_SDK_ROOT
[ -n ${ANDROID_SDK_ROOT} ] && unset ANDROID_HOME
echo $ANDROID_SDK_ROOT

rename_tab drishti $TOOLCHAIN

COMMAND=(
    "--verbose "
    "--fwd ANDROID=TRUE "
    "${DRISHTI_BUILD_ARGS[*]} "
    "${DRISHTI_BUILD_HIDE[*]} "
    "${DRISHTI_BUILD_LOCAL[*]} "
    "${DRISHTI_POLLY_ARGS[*]} "
    "--jobs 8 "
#    "--strip "
)

COMMAND+=( $(add_polly_commands "$@") )

do_launch=0
args_count=${#COMMAND[@]}
echo ${args_counts}
if [ ${args_count} -gt 0 ]; then
    option=${COMMAND[${args_count}-1]}
    if [[ ${option} =~ ^--launch.* ]]; then
        unset 'COMMAND[${args_count}-1]'
        echo "DO_LAUNCH"
        do_launch=1
    fi
fi

eval polly.py --toolchain ${TOOLCHAIN} ${COMMAND[*]}

# Launch application (optional)
android_build_path=${DRISHTISDK}/_builds/${TOOLCHAIN}-${DRISHTI_BUILD_CONFIG}/
android_makefile=${android_build_path}/Makefile
android_targets=( $(grep -o -e ".*launch:" ${android_makefile} | tr -d ':') )
echo "do_launch=${do_launch}"
if [ ${#android_targets[@]} -gt 0 ] && [ ${do_launch} -gt 0 ]; then
    # run the first launch taret
    ( cd ${android_build_path} && make ${android_targets[0]} )
fi

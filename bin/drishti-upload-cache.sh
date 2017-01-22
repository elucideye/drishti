#!/bin/bash
#
# Upload hunter cache to server
#

if [ -z "${DRISHTISDK}" ]; then
    echo 2>&1 "Must have DRISHTISDK set"
fi

if [ -z "${DRISHTISDK_BOT_USER}" ]; then
    echo 2>&1 "Must have DRISHTISDK_BOT set"
fi

if [ -z "${DRISHTISDK_REPO}" ]; then
    echo 2>&1 "Must have DRISHTISDK_REPO set"
fi

if [ -z "${DRISHTISDK_REPO_OWNER}" ]; then
    echo 2>&1 "Must have DRISHTISDK_REPO_OWNER set"
fi

if [ -z "${GITHUB_USER_PASSWORD}" ]; then
    echo 2>&1 "Must have GITHUB_USER_PASSWORD set"
fi

if [ -z "${HUNTER_ROOT}" ]; then
    echo 2>&1 "Must have HUNTER_ROOT set"
fi

ARGS=(
    "--username ${DRISHTISDK_BOT} "
    "--repo-owner ${DRISHTISDK_REPO_OWNER} "
    "--repo ${DRISHTISDK_REPO} "
    "--cache-dir ${HUNTER_ROOT}/_Base/Cache "
    "--temp-dir ${HOME}/__TEMP_HUNTER "
)

python3 ${HUNTER_ROOT}/maintenance/upload-cache-to-github.py ${ARGS[@]}

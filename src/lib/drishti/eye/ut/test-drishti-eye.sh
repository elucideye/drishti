CMAKE_SOURCE_DIR=${DRISHTISDK}

TESTDIR=$(find ${CMAKE_SOURCE_DIR}/_builds -name "drishti-assets-1.0")


NAME=2318-eye

ARGS=(
    "${TESTDIR}/drishti_eye_full_npd_eix.pba.z"
    "${TESTDIR}/images/${NAME}.png"
    "${TESTDIR}/images/${NAME}.json"
)

echo ${ARGS[@]}


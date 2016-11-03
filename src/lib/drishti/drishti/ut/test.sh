CMAKE_SOURCE_DIR=${DRISHTISDK}

NAME=2318-eye

# TODO
ARGS=(
    "${CMAKE_SOURCE_DIR}/assets/drishti_eye_full_npd_eix.pba.z"
    "${CMAKE_SOURCE_DIR}/assets/images/${NAME}.png"
    "${CMAKE_SOURCE_DIR}/assets/images/${NAME}.json"
)

echo ${ARGS[@]}

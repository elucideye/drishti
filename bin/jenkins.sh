#!/bin/bash -e

# http://releases.llvm.org/6.0.1/clang+llvm-6.0.1-x86_64-linux-gnu-ubuntu-14.04.tar.xz
CLANG_LLVM_NAME="clang+llvm-6.0.1-x86_64-linux-gnu-ubuntu-14.04"
CLANG_LLVM_FILENAME="${CLANG_LLVM_NAME}.tar.xz"
CLANG_LLVM_URL="http://releases.llvm.org/6.0.1"
CLANG_LLVM_FULL_URL="${CLANG_LLVM_URL}/${CLANG_LLVM_FILENAME}"

_THIS_SOURCE_DIRNAME=`dirname "${BASH_SOURCE[0]}"`
cd "${_THIS_SOURCE_DIRNAME}/.."

if [[ "${TRAVIS}" == "true" ]];
then
  echo "Travis CI configuration"
else
  echo "Local build configuration"
fi

# Find all internal files, making sure to exlude 3rdparty subprojects
function find_source()
{
    file_ext=(-name "*.h" -or -name "*.cpp" -or -name "*.hpp" -or -name "*.java")
    find $1 -not \( -path `pwd`/src/3rdparty -prune \) ${file_ext[@]}
}

case "${TYPE}" in
  "polly")
    if [[ "${CONFIG}" == "" ]];
    then
      echo "CONFIG is not set"
      exit 1
    fi

    if [[ "${TOOLCHAIN}" == "" ]];
    then
      echo "TOOLCHAIN is not set"
      exit 1
    fi

    if [[ "${INSTALL}" == "" ]];
    then
      echo "INSTALL is not set"
      exit 1
    fi

    if [[ "${TOOLCHAIN}" == ios-* ]];
    then
      if [[ "${TRAVIS}" == "true" ]];
      then
        TEST=""
      else
        TEST="--test"
      fi
    else
      TEST="--test"
    fi

    if [[ "${TOOLCHAIN}" == android-* ]];
    then
      GPU=ON
    else
      if [[ "${TRAVIS}" == "true" ]];
      then
        GPU=OFF
      else
        GPU=ON
      fi
    fi

    if [[ "${TOOLCHAIN}" == "clang-tidy-libcxx" ]];
    then
      SHARED_SDK=OFF
      if [[ "${TRAVIS}" == "true" ]];
      then
        VERBOSE=""

        wget -q "${CLANG_LLVM_FULL_URL}"
        tar xf "${CLANG_LLVM_FILENAME}"
        export PATH="`pwd`/${CLANG_LLVM_NAME}/bin:${PATH}"

        # Avoid checks in CMake sources, e.g. compiling ABI info
        echo "---" >> _ci/.clang-tidy
        echo "Checks: '-*'" >> _ci/.clang-tidy
        echo "WarningsAsErrors: ''" >> _ci/.clang-tidy
      else
        VERBOSE=--verbose
      fi
    else
      SHARED_SDK=ON
      VERBOSE=--verbose
    fi

    if [[ ${TRAVIS} == "true" ]]; then
      GAUZE_ANDROID_USE_EMULATOR=YES # remote test w/ emulator
    else
      GAUZE_ANDROID_USE_EMULATOR=NO # support local host testing on a real device
    fi

    if [[ `uname` == "Linux" ]]; then
      GAUZE_ANDROID_EMULATOR_GPU=swiftshader
    else
      GAUZE_ANDROID_EMULATOR_GPU=host
    fi

    set -x

    polly.py \
        --toolchain "${TOOLCHAIN}" \
        --config "${CONFIG}" \
        ${VERBOSE} \
        --ios-multiarch --ios-combined \
        --archive drishti \
        --jobs 4 \
        ${TEST} \
        "${INSTALL}" \
        --fwd \
        DRISHTI_BUILD_SHARED_SDK=${SHARED_SDK} \
        DRISHTI_BUILD_TESTS=YES \
        DRISHTI_BUILD_EXAMPLES=YES \
        DRISHTI_COPY_3RDPARTY_LICENSES=ON \
        DRISHTI_HAS_GPU=${GPU} \
        DRISHTI_HUNTER_CONFIG_MINIMAL=${MINIMAL} \
        GAUZE_ANDROID_USE_EMULATOR=${GAUZE_ANDROID_USE_EMULATOR} \
        GAUZE_ANDROID_EMULATOR_GPU=${GAUZE_ANDROID_EMULATOR_GPU} \
        GAUZE_ANDROID_EMULATOR_PARTITION_SIZE=40 \
        CMAKE_CONFIGURATION_TYPES="${CONFIG}" \
        HUNTER_CONFIGURATION_TYPES="${CONFIG}" \
        HUNTER_SUPPRESS_LIST_OF_FILES=ON \
        DRISHTI_CODE_SIGN=OFF

    ;;
  "android-studio")

    case "${ANDROID_STUDIO_ARCH}" in
      "x86_64")
        ;;
      "armeabi-v7a")
        ;;
      "arm64-v8a")
        ;;
      "")
        echo "ANDROID_STUDIO_ARCH is not set, try:"
        echo "  x86_64"
        echo "  armeabi-v7a"
        echo "  arm64-v8a"
        exit 1
        ;;
      *)
        echo "Unknown ANDROID_STUDIO_ARCH value: '${ANDROID_STUDIO_ARCH}'"
        exit 1
        ;;
    esac

    if [[ "${TRAVIS}" != "true" ]];
    then
      cd android-studio

      expected="`pwd`/local.properties"

      if [ ! -f "${expected}" ];
      then
        echo "File '${expected}' not found"
        exit 1
      fi

      set -x

      ../bin/travis_wait -i 60 "./gradlew assembleDebug -Parch=${ANDROID_STUDIO_ARCH}"

      exit 0
    fi

    export CMAKE_DIR="`pwd`/_ci/cmake"

    mkdir _ninja
    cd _ninja
    wget -q "https://github.com/ninja-build/ninja/releases/download/v1.8.2/ninja-linux.zip"
    unzip -q ninja-linux.zip
    export PATH="`pwd`:${PATH}"
    cd ..

    wget -q "https://dl.google.com/android/repository/android-ndk-r18b-linux-x86_64.zip"
    unzip -q android-ndk-r18b-linux-x86_64.zip
    export ANDROID_NDK="`pwd`/android-ndk-r18b"
    rm android-ndk-r18b-linux-x86_64.zip

    cd android-studio

    # Create 'local.properties'
    echo "sdk.dir=${HOME}/android-sdk" >> local.properties
    echo "ndk.dir=${ANDROID_NDK}" >> local.properties
    echo "cmake.dir=${CMAKE_DIR}" >> local.properties

    # https://stackoverflow.com/a/38381577
    mkdir -p ${HOME}/android-sdk/licenses
    echo -e "\n24333f8a63b6825ea9c5514f83c2829b004d1fee" > "${HOME}/android-sdk/licenses/android-sdk-license"

    # https://stackoverflow.com/a/38339046
    echo "android.builder.sdkDownload=true" > gradle.properties

    set -x

    # Let's stop Gradle build at beginning of CMakeLists.txt so we can lauch
    # it explicitly by `cmake --build` (workaround for Android Studio bug, see below)
    sed -i 's,^if(DRISHTI_DEBUG_STOP)$,if(DRISHTI_DEBUG_STOP OR TRUE),' ../CMakeLists.txt

    # First Gradle lauch will hit issue:
    # * https://issuetracker.google.com/issues/75268076
    ./gradlew assembleDebug -Parch=${ANDROID_STUDIO_ARCH} || echo "Ooops"

    # Sometimes second launch failing with the same error, put a wait command
    # to try to improve stability (empirical note: 30 is not enough)
    sleep 45

    # Now should be fine
    ../bin/travis_wait -i 60 "./gradlew assembleDebug -Parch=${ANDROID_STUDIO_ARCH}"

    # Back to normal CMake configuration
    sed -i 's,^if(DRISHTI_DEBUG_STOP OR TRUE)$,if(DRISHTI_DEBUG_STOP),' ../CMakeLists.txt

    # Lauch CMake build without Gradle.
    ../bin/travis_wait -i 60 "cmake --build ../src/examples/facefilter/android-studio/app/.externalNativeBuild/cmake/debug/${ANDROID_STUDIO_ARCH}"

    # CMake part done, now we can continue with the
    # rest parts of Android Studio project
    ./gradlew assembleDebug -Parch=${ANDROID_STUDIO_ARCH}

    ;;
  "clang-format-check")

    if [[ "${TRAVIS}" == "true" ]];
    then
      wget -q "${CLANG_LLVM_FULL_URL}"
      tar xf "${CLANG_LLVM_FILENAME}"
      CLANG_FORMAT="`pwd`/${CLANG_LLVM_NAME}/bin/clang-format"
    else
      CLANG_FORMAT=clang-format
    fi

    input_dir=`pwd`

    echo "Run clang-format for directory (check only): '${input_dir}'"

    tempfile=/tmp/__clang_format_check.txt

    find_source ${input_dir} | grep -v -e '/src/3rdparty/' -e '/_builds/' -e '/polly-master/examples/' -e '/_ci/' -e '/clang.llvm-6.0.1-x86_64-linux-gnu-ubuntu-14.04/' | while read name
    do
        echo "Check file: '${name}'"
        ${CLANG_FORMAT} -style=file ${name} > ${tempfile}
        diff "${name}" "${tempfile}"
    done

    ;;
  "clang-format-run")
    input_dir=`pwd`

    echo "Run clang-format for directory (modify): '${input_dir}'"

    find_source ${input_dir} | grep -v -e '/src/3rdparty/' -e '/_builds/' | while read name
    do
        echo "Check file: '${name}'"
        clang-format -i -style=file ${name}
    done

    ;;
  "")
    echo "TYPE is not set, try:"
    echo "  polly"
    echo "  android-studio"
    echo "  clang-format-check"
    echo "  clang-format-run"
    exit 1
    ;;
  *)
    echo "Unknown TYPE value: '${TYPE}'"
    exit 1
    ;;
esac

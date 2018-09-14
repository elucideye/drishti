import os
import subprocess
import sys

from detail.download_unzip import download_unzip
from detail.substitute_line import substitute_line

def android_studio_build():
  arch = os.getenv('ANDROID_STUDIO_ARCH')

  arch_expected = 'Expected values:\n* x86_64\n* armeabi-v7a\n* arm64-v8a'

  if not arch:
    sys.exit('Environment variable ANDROID_STUDIO_ARCH is empty.\n{}'.format(arch_expected))

  if arch != 'x86_64' and arch != 'armeabi-v7a' and arch != 'arm64-v8a':
    sys.exit('Unknown ANDROID_STUDIO_ARCH value: "{}".\n{}'.format(arch, arch_expected))

  is_appveyor = (os.getenv('APPVEYOR') == 'True')

  cmake_dir = os.path.join(os.getcwd(), '_ci', 'cmake')
  cmake_build_directory = os.path.join(
      os.getcwd(),
      'src',
      'examples',
      'facefilter',
      'android-studio',
      'app',
      '.externalNativeBuild',
      'cmake',
      'debug',
      arch
  )

  cmakelists_top = os.path.join(os.getcwd(), 'CMakeLists.txt')
  if not os.path.exists(cmakelists_top):
    sys.exit('File not found: "{}"'.format(cmakelists_top))

  os.chdir('android-studio')

  if is_appveyor:
    download_unzip(
        'https://github.com/ninja-build/ninja/releases/download/v1.8.2/ninja-win.zip',
        '_ninja'
    )

    ninja_path = os.path.join(os.getcwd(), '_ninja')
    os.environ['PATH'] = "{};{}".format(ninja_path, os.getenv('PATH'))

    download_unzip(
        'https://dl.google.com/android/repository/android-ndk-r17-linux-x86_64.zip',
        '_android_ndk'
    )

    android_ndk = os.path.join(os.getcwd(), '_android_ndk', 'android-ndk-r17')

    f = open('local.properties', 'w')
    f.write('sdk.dir={}/android-sdk\n'.format(os.getenv('USERPROFILE')))
    f.write('ndk.dir={}\n'.format(android_ndk))
    f.write('cmake.dir={}\n'.format(cmake_dir))
    f.close()

  expected = os.path.join(os.getcwd(), 'local.properties')

  if not os.path.exists(expected):
    sys.exit('Path not found: {}'.format(expected))

  # Let's stop Gradle build at beginning of CMakeLists.txt so we can lauch
  # it explicitly by `cmake --build` (workaround for Android Studio bug, see below)
  substitute_line(
      cmakelists_top,
      '^if\(DRISHTI_DEBUG_STOP\)$',
      'if(DRISHTI_DEBUG_STOP OR TRUE)'
  )

  subprocess.check_call(
      ['gradlew.bat', 'assembleDebug', '-Parch={}'.format(arch)]
  )

  if not os.path.exists(cmake_build_directory):
    sys.exit('Path not found: {}'.format(cmake_build_directory))

  # Back to normal CMake configuration
  substitute_line(
      cmakelists_top,
      '^if\(DRISHTI_DEBUG_STOP OR TRUE\)$',
      'if(DRISHTI_DEBUG_STOP)'
  )

  # Lauch CMake build without Gradle.
  subprocess.check_call(['cmake', '--build', cmake_build_directory])

  # CMake part done, now we can continue with the
  # rest parts of Android Studio project
  subprocess.check_call(
      ['gradlew.bat', 'assembleDebug', '-Parch={}'.format(arch)]
  )

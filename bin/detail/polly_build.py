import os
import subprocess
import sys
import time

from detail.download_unzip import download_unzip

def run():
  polly_script = [sys.executable]

  if os.getenv('APPVEYOR') == 'True':
    download_unzip(
        'https://github.com/ruslo/polly/archive/master.zip', '_polly'
    )

    cwd = os.getcwd()
    polly_bin = os.path.join(cwd, '_polly', 'polly-master', 'bin')

    # Install dependencies (CMake, Ninja)
    ci_deps_script = os.path.join(polly_bin, 'install-ci-dependencies.py')
    subprocess.check_call([sys.executable, ci_deps_script])

    # Tune locations
    cmake_bin = os.path.join(cwd, '_ci', 'cmake', 'bin')
    os.environ['PATH'] = "{};{}".format(cmake_bin, os.getenv('PATH'))

    polly_script += [os.path.join(polly_bin, 'polly.py')]
  else:
    polly_script += [subprocess.check_output(['where', 'polly.py']).decode('utf-8').rstrip()]

  config = os.getenv('CONFIG')
  if not config:
    sys.exit('Environment variable CONFIG is empty')

  toolchain = os.getenv('TOOLCHAIN')
  if not toolchain:
    sys.exit('Environment variable TOOLCHAIN is empty')

  build_shared = os.getenv('BUILD_SHARED')
  if not build_shared:
    sys.exit('Environment variable BUILD_SHARED is empty')

  polly_script += [
      '--verbose',
      '--archive',
      'drishti',
      '--config',
      config,
      '--toolchain',
      toolchain,
      '--test',
      '--fwd',
      'DRISHTI_BUILD_SHARED_SDK={}'.format(build_shared),
      'DRISHTI_COPY_3RDPARTY_LICENSES=ON',
      'DRISHTI_BUILD_TESTS=ON',
      'DRISHTI_BUILD_EXAMPLES=ON',
      'DRISHTI_HAS_GPU=OFF'
  ]

  if os.getenv('APPVEYOR') == 'True':
    polly_script += ['HUNTER_SUPPRESS_LIST_OF_FILES=ON']

  subprocess.check_call(polly_script)

import os
import requests
import subprocess
import sys
import time

# http://stackoverflow.com/a/16696317/2288008
def file_download_once(url, local_path):
  print('Downloading:\n  {}\n  -> {}'.format(url, local_path))
  r = requests.get(url, stream=True)
  if not r.ok:
    raise Exception('Downloading failed: {}'.format(url))
  with open(local_path, 'wb') as f:
    for chunk in r.iter_content(chunk_size=16*1024):
      if chunk:
        f.write(chunk)

def file_download(url, local_path):
  max_retry = 3
  for i in range(max_retry):
    try:
      file_download_once(url, local_path)
      print('Done')
      return
    except Exception as exc:
      print('Exception catched ({}), retry... ({} of {})'.format(exc, i+1, max_retry))
      time.sleep(60)
  sys.exit('Download failed')

def run():
  polly_script = [sys.executable]

  if os.getenv('APPVEYOR') == 'True':
    # Install latest Polly toolchains and scripts
    cwd = os.getcwd()
    polly_local_path = os.path.join(cwd, 'polly.zip')
    file_download(
        'https://github.com/ruslo/polly/archive/master.zip', polly_local_path
    )

    # Can't use ZipFile module because permissions will be lost, see bug:
    # * https://bugs.python.org/issue15795
    devnull = open(os.devnull, 'w') # subprocess.DEVNULL is not available for Python 3.2
    subprocess.check_call(
        ['7z', 'x', '-y', polly_local_path], stdout=devnull, stderr=devnull, bufsize=0
    )

    polly_bin = os.path.join(cwd, 'polly-master', 'bin')

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

  subprocess.check_call(polly_script)

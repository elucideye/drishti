import os
import subprocess
import sys

from detail.android_studio_build import android_studio_build
from detail.download_unzip import download_unzip
from detail.polly_build import polly_build

ci_type = os.getenv('TYPE')

ci_type_expected = 'Expected values:\n* polly\n* android-studio'

if ci_type == None:
  sys.exit('TYPE is empty.\n{}'.format(ci_type_expected))

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

if ci_type == 'polly':
  polly_build()
elif ci_type == 'android-studio':
  android_studio_build()
else:
  sys.exit('Unknown TYPE value: "{}".\n{}'.format(ci_type, ci_type_expected))

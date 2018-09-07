import os
import sys

import detail.android_studio_build
import detail.polly_build

ci_type = os.getenv('TYPE')

ci_type_expected = 'Expected values:\n* polly\n* android-studio'

if ci_type == None:
  sys.exit('TYPE is empty.\n{}'.format(ci_type_expected))

if ci_type == 'polly':
  detail.polly_build.run()
elif ci_type == 'android-studio':
  detail.android_studio_build.run()
else:
  sys.exit('Unknown TYPE value: "{}".\n{}'.format(ci_type, ci_type_expected))

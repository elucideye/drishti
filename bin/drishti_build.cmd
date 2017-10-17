:: Name: drishti_build.cmd
:: Purpose: Support readable multi-line polly.py build commands
:: Copyright 2016 Elucideye, Inc.
::
:: Multi-line commands are not currently supported directly in appveyor.yml files
::
:: See: http://stackoverflow.com/a/37647169
::
:: This script is identical to build-appveyor.cmd, however it allows
:: build from source for new toolchains.

echo POLLY_ROOT %POLLY_ROOT%

python %POLLY_ROOT%\bin\polly.py ^
--verbose ^
--archive drishti ^
--config "%1%" ^
--toolchain "%2%" ^
--test ^
--fwd HUNTER_USE_CACHE_SERVERS=YES ^
HUNTER_DISABLE_BUILDS=NO ^
DRISHTI_COPY_3RDPARTY_LICENSES=ON ^
DRISHTI_BUILD_TESTS=ON ^
DRISHTI_BUILD_EXAMPLES=ON

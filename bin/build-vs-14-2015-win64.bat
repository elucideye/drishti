:: Seet POLLY_ROOT, HUNTER_ROOT and PATH for PYTHON on Windows Platform
::
:: Reference:
:: set PATH=c:\\Users\dhirv\AppData\\Local\\Programs\\Python\\Python35;c:\\Program Files\\CMake\\bin;%PATH%
:: set POLLY_ROOT=%cd%\..\polly
:: set HUNTER_ROOT=%cd%\..\hunter

set CONFIG=Debug
set TOOLCHAIN=vs-14-2015-win64
python %POLLY_ROOT%\bin\polly.py --reconfig ^
--verbose ^
--archive drishti ^
--config "%CONFIG%" ^
--toolchain "%TOOLCHAIN%" ^
--test ^
--fwd HUNTER_USE_CACHE_SERVERS=YES ^
DRISHTI_USE_TEXT_ARCHIVES=YES  ^
DRISHTI_BUILD_TESTS=ON ^
DRISHTI_BUILD_EXAMPLES=ON ^
--open --reconfig
::--nobuild -open

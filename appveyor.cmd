:: Name: appveyor.cmd
:: Purpose: Support readable multi-line polly.py build commands
:: Copyright 2016 Elucideye, Inc.
::
:: Multi-line commands are not currently supported directly in appveyor.yml files
::
:: See: http://stackoverflow.com/a/37647169

set PATH=C:\Python34-x64;C:\Python34-x64\Scripts;%PATH%

:: Tune locations
set PATH=%cd%\_ci\cmake\bin;%PATH%
set PATH=%cd%\_ci\ninja;%PATH%

:: Remove entry with sh.exe from PATH to fix error with MinGW toolchain
:: (For MinGW make to work correctly sh.exe must NOT be in your path)
:: * http://stackoverflow.com/a/3870338/2288008
set PATH=%PATH:C:\Program Files\Git\usr\bin;=%

:: Use MinGW from Qt tools because version is higher
:: * http://www.appveyor.com/docs/installed-software#qt
set MINGW_PATH=C:\Qt\Tools\mingw492_32\bin

:: MSYS2 location
set MSYS_PATH=C:\msys64\usr\bin

set POLLY_CMD=%1
set CONFIG=%2
set TOOLCHAIN=%3
python %POLLY_CMD% ^
--verbose ^
--pack TGZ ^
--config "%CONFIG%" ^
--toolchain "%TOOLCHAIN%" ^
--test ^
--fwd HUNTER_USE_CACHE_SERVERS=YES DRISHTI_USE_TEXT_ARCHIVES=YES

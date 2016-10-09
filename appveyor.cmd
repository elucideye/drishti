:: Name: appveyor.cmd
:: Purpose: Support readable multi-line polly.py build commands
:: Copyright 2016 Elucideye, Inc.
::
:: Multi-line commands are not currently supported directly in appveyor.yml files
::
:: See: http://stackoverflow.com/a/37647169

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

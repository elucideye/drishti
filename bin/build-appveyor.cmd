:: Name: build-appveyor.cmd
:: Purpose: Support readable multi-line polly.py build commands
:: Copyright 2016 Elucideye, Inc.
::
:: Multi-line commands are not currently supported directly in appveyor.yml files
::
:: See: http://stackoverflow.com/a/37647169

python polly.py ^
--verbose ^
--pack TGZ ^
--config "%1%" ^
--toolchain "%2%" ^
--test ^
--fwd HUNTER_USE_CACHE_SERVERS=ONLY ^
HUNTER_DISABLE_BUILDS=YES ^
DRISHTI_USE_TEXT_ARCHIVES=YES

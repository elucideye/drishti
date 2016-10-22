python %POLLY_ROOT%\bin\polly.py ^
--verbose ^
--pack TGZ ^
--config "%CONFIG%" ^
--toolchain "%TOOLCHAIN%" ^
--test ^
--fwd HUNTER_USE_CACHE_SERVERS=YES DRISHTI_USE_TEXT_ARCHIVES=YES --install


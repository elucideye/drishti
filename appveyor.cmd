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

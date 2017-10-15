:: Name: hunter_env.cmd
:: Purpose: Polly + cmake installation for hunter development
:: Copyright 2017 Elucideye, Inc.
::
:: Source: https://github.com/ingenue/hunter/blob/pkg.template/appveyor.yml

:: Python 3
set PATH=C:\Python34-x64;C:\Python34-x64\Scripts;%PATH%

:: Install Python package 'requests'
pip install requests

:: Install latest Polly toolchains and scripts
appveyor DownloadFile https://github.com/ruslo/polly/archive/master.zip
7z x master.zip
set POLLY_ROOT=%cd%\polly-master

:: Install dependencies (CMake, Ninja)
python %POLLY_ROOT%\bin\install-ci-dependencies.py

:: Tune locations
set PATH=%cd%\_ci\cmake\bin;%PATH%
set PATH=%cd%\_ci\ninja;%PATH%

:: Add '--quiet' to avoid leaking the token to logs
git submodule update --init --recursive --quiet

:: Remove entry with sh.exe from PATH to fix error with MinGW toolchain
:: (For MinGW make to work correctly sh.exe must NOT be in your path)
:: * http://stackoverflow.com/a/3870338/2288008
set PATH=%PATH:C:\Program Files\Git\usr\bin;=%

:: Use MinGW from Qt tools because version is higher
:: * http://www.appveyor.com/docs/installed-software#qt
set MINGW_PATH=C:\Qt\Tools\mingw492_32\bin

:: MSYS2 location
set MSYS_PATH=C:\msys64\usr\bin

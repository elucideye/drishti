# drishti

[![Travis][travis_shield]][travis_builds]
[![Appveyor][appveyor_shield]][appveyor_builds]
[![License (3-Clause BSD)][license_shield]][license_link]
[![Hunter][hunter_shield]][hunter_link]
[![Gitter][gitter_badge]][gitter_link]

[gitter_badge]: https://badges.gitter.im/elucideye/drishti.svg
[gitter_link]: https://gitter.im/elucideye/drishti?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge

[hunter_shield]: https://img.shields.io/badge/hunter-v0.19.94-blue.svg
[hunter_link]: http://github.com/ruslo/hunter

[license_shield]: https://img.shields.io/badge/license-BSD%203--Clause-brightgreen.svg?style=flat-square
[license_link]: http://opensource.org/licenses/BSD-3-Clause

[travis_builds]: https://travis-ci.org/elucideye/drishti/builds
[travis_shield]: https://img.shields.io/travis/elucideye/drishti/master.svg?style=flat-square&label=Linux%20OSX%20Android%20iOS

[appveyor_builds]: https://ci.appveyor.com/project/headupinclouds/drishti
[appveyor_shield]: https://img.shields.io/appveyor/ci/headupinclouds/drishti.svg?style=flat-square&label=Windows 

![drishti_text_big](https://user-images.githubusercontent.com/554720/28922218-3a005f9c-7827-11e7-839c-ef3e9a282f70.png)

### Real time eye tracking for embedded and mobile devices in C++11.

![eye models 1](https://user-images.githubusercontent.com/554720/28920911-d836e56a-7821-11e7-8b41-bc338f100cc1.png)
![eye models 2](https://user-images.githubusercontent.com/554720/28920912-da9f3820-7821-11e7-848c-f526922e24ec.png)
![eye models 3](https://user-images.githubusercontent.com/554720/28920920-dcd8e708-7821-11e7-8fc2-b9f375a9a550.png)

Goal: SDK size <= 1 MB and combined resources (object detection + regression models) <= 4 MB.

* [Hunter](https://github.com/ruslo/hunter) package management and CMake build system by Ruslan Baratov, as well as much of the cross platform Qt work: "Organized Freedom!" :)
* A C++ and OpenGL ES 2.0 implementation of [Fast Feature Pyramids for Object Detection](https://pdollar.github.io/files/papers/DollarPAMI14pyramids.pdf) (see [Piotr's Matlab Toolbox](https://pdollar.github.io/toolbox)) for face and eye detection
* Iris ellipse fitting via [Cascaded Pose Regression](https://pdollar.github.io/files/papers/DollarCVPR10pose.pdf) (Piotr Dollar, et al) + [XGBoost](https://github.com/dmlc/xgboost) regression (Tianqi Chen, et al) 
* Face landmarks and eye contours provided by ["One Millisecond Face Alignment with an Ensemble of Regression Trees"](http://www.cv-foundation.org/openaccess/content_cvpr_2014/papers/Kazemi_One_Millisecond_Face_2014_CVPR_paper.pdf) (Kazemi, et al) using a modified implementation from [Dlib](https://github.com/davisking/dlib) (Davis King) (normalized pixel differences, line indexed features, PCA size reductions)
* OpenGL ES friendly GPGPU shader processing and efficient iOS + Android texture handling using a modified version of [ogles_gpgpu](https://github.com/hunter-packages/ogles_gpgpu) (Markus Kondrad) with a number of shader implementations taken directly from [GPUImage](https://github.com/BradLarson/GPUImage) (Brad Larson)

iPhone @ 30 FPS (VIDEO)                                        |  Hitchcock 
:-------------------------------------------------------------:|:-------------------------:
[![iPhone](https://goo.gl/1uLQ44)](https://vimeo.com/230351171)|[![hitchcock](https://goo.gl/rMzrBB)](https://vimeo.com/219386623)

* Hithcock source: Koganada's "Eyes of Hitchcock"*

## Quick Start (i.e., How do I make this library work?)

Drishti is a [CMake](https://github.com/kitware/CMake) based project that uses the [Hunter](https://github.com/ruslo/hunter) package manager to download and build project dependencies from source as needed.  Hunter contains [detailed documentation](https://docs.hunter.sh/en/latest), but a few high level notes and documentation links are provided here to help orient first time users.  In practice, some working knowledge of CMake may also be required.  Hunter itself is written in CMake, and is installed as part of the build process from a single `HunterGate()` macro at the top of the root `CMakeLists.txt` file (typically `cmake/Hunter/HunterGate.cmake`) (you don't have to build or install it).  Each CMake dependency's `find_package(FOO)` call that is paired with a `hunter_add_package(FOO CONFIG REQUIRED)` will be managed by Hunter.  In most cases, the only system requirement for building a Hunter project is a recent [CMake with CURL](https://docs.hunter.sh/en/latest/contributing.html#reporting-bugs) support and a working compiler correpsonding to the operative toolchain.  Hunter will maintain all dependencies in a [versioned](https://docs.hunter.sh/en/latest/overview/customization.html) local [cache](https://docs.hunter.sh/en/latest/overview/shareable.html) by default (typically `${HOME}/.hunter`) where they can be reused in subsequent builds and shared between different projects.  They can also be stored in a server side [binary cache](https://docs.hunter.sh/en/latest/overview/binaries.html) -- select [toolchains](#Toolchains) will be backed by a server side binary cache (https://github.com/elucideye/hunter-cache) and will produce faster first time builds (use them if you can!).

The [Travis](https://github.com/elucideye/drishti/blob/master/.travis.yml) (Linux/OSX/iOS/Android) and [Appveyor](https://github.com/elucideye/drishti/blob/master/appveyor.yml) (Windows) CI scripts in the project's root directory can serve as a reference for basic setup when building from source.  To support cross platform builds and testing, the CI scripts make use of [Polly](https://github.com/ruslo/polly): a set of common CMake toolchains paired with a simple `polly.py` CMake build script.  Polly is used here for convenience to generate `CMake` command line invocations -- it is not required for building Hunter projects.

To reproduce the CI builds on a local host, the following setup is recommended:

* Install compiler: http://cgold.readthedocs.io/en/latest/first-step.html
* Install [CMake](https://github.com/kitware/CMake) (and add to `PATH`)
* Install Python (for Polly)
* Clone [Polly](https://github.com/ruslo/polly) and add `<polly>/bin` to `PATH`

Note: Polly is not a build requirement, CMake can always be used directly, but it is used here for convenience.

The `bin/hunter_env.{sh,cmd}` scripts (used in the CI builds) can be used as a fast shortcut to install these tools for you.  You may want to add the `PATH` variables permanently to your `.bashrc` file (or equivalent) for future sessions.

| Linux/OSX/Android/iOS         | Windows                   |
|-------------------------------|---------------------------|
| `source bin/hunter_env.sh`    | `bin\hunter_env.cmd`      |

After the environment is configured, you can build for any supported `Polly` toolchain (see `polly.py --help`) with a command like this:

```
polly.py --toolchain ${TOOLCHAIN} --config ${CONFIG} --fwd HUNTER_CONFIGURATION_TYPES=${CONFIG} --install --verbose
```

## Toolchains 

The configurations listed below have all been tested.  In general, most C++11 toolchains should work with minimal effort.  A `CI` comment indicates that the configuration is part of the Travis or Appveyor CI tests, so all Hunter packages will be available in the server side binary cache.

Linux (Ubunty Trusty 14.04):
* `TOOLCHAIN=gcc-5-pic-hid-sections-lto` `CONFIG=Release` # CI
* `TOOLCHAIN=libcxx` `CONFIG=Release` # w/ clang 3.8

OSX:
* `TOOLCHAIN=osx-10-11-hid-sections-lto` `CONFIG=Release` # CI
* `TOOLCHAIN=osx-10-12-sanitize-address-hid-sections` `CONFIG=Release` # CI
* `TOOLCHAIN=xcode-hid-sections` `CONFIG=Release` # generic

iOS:
* `TOOLCHAIN=ios-nocodesign-10-1-arm64-dep-9-0-device-libcxx-hid-sections-lto` `CONFIG=MinSizeRel` # CI
* `TOOLCHAIN=ios-10-1-arm64-dep-8-0-hid-sections` `CONFIG=Release`

Android (from OSX):
* `TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections` `CONFIG=MinSizeRel` # CI
* `TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections-lto` `CONFIG=MinSizeRel`

Windows:
* `TOOLCHAIN=vs-14-2015-sdk-8-1` `CONFIG=Release` # CI
* `TOOLCHAIN=vs-14-2015-sdk-8-1` `CONFIG=Debug` # CI
* `TOOLCHAIN=vs-14-2015-win64-sdk-8-1` `CONFIG=Release` # CI
* `TOOLCHAIN=vs-14-2015-win64-sdk-8-1` `CONFIG=Debug` # CI

The polly out of source build trees are located in `_builds/${TOOLCHAIN}`, the final build products (the stuff you want) are installed in `_install/${TOOLCHAIN}`, and the build logs are dumped in `_logs/${TOOLCHAIN}`.  The iOS frameworks are installed in `_frameworks/${TOOLCHAIN}`. 


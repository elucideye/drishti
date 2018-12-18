drishti
=======

|Travis| |Appveyor| |License (3-Clause BSD)| |Hunter| |Gitter|

.. figure:: https://user-images.githubusercontent.com/554720/28922218-3a005f9c-7827-11e7-839c-ef3e9a282f70.png
   :alt: drishti\_text\_big

Real time eye tracking for embedded and mobile devices in C++11.
================================================================

|eye models 1| |eye models 2| |eye models 3|

NEWS (2018/08/10)
-----------------

Native iOS, Android, and "desktop" variants of the real-time
``facefilter`` application have been added here:
`src/examples/facefilter <https://github.com/elucideye/drishti/tree/master/src/examples/facefilter>`__.
These applications link against the installed public ``drishti::drishti``
package interface, which is designed without external types in the API definition.
The ``facefilter`` demos are enabled by the ``DRISHTI_BUILD_EXAMPLES``
CMake option, and the entire ``src/examples`` tree is designed to be relocatable,
you can ``cp -r src/examples ${HOME}/drishti_examples``, customize, and
build, by simply updating the drishti package details.

iOS
~~~

The ``iOS`` ``facefilter`` target requires Xcode 9 (beta 4) or above
(Swift language requirements)  and will be generated directly as a standard CMake
``add_executable()`` target as part of the usual top level project build --
*if* you are using an appropriate CMake
`iOS toolchain <https://polly.readthedocs.io/en/latest/toolchains/ios.html>`__
for
`cross compilation <https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/CrossCompiling>`__
from your macOS + Xcode host for your ``iOS`` device.   Please see
`Polly Based Build`_ and `iOS Build`_ below for more details.

Android Studio
~~~~~~~~~~~~~~

The top level Android Studio application is located in the ``android-studio`` directory.
This target will build and manage repository C++ sources directly as part of the project.
Android Studio/Gradle is required to build the application layer,
and the CMake build is managed directly by ``gradle``.  There are a
few platform specific configurations that must be addressed before building.
Please see `Android Studio Build`_ below for more details.

Overview
--------

Goal: SDK size <= 1 MB and combined resources (object detection +
regression models) <= 4 MB.

-  `Hunter <https://github.com/ruslo/hunter>`__ package management and
   CMake build system by Ruslan Baratov, as well as CI and much of the
   real time `facefilter` mobile application(s) layer: "Organize Freedom!" :)
-  A C++ and OpenGL ES 2.0 implementation of
   `Fast Feature Pyramids for Object Detection
   <https://pdollar.github.io/files/papers/DollarPAMI14pyramids.pdf>`__
   (see `Piotr's Matlab Toolbox <https://pdollar.github.io/toolbox>`__)
   for face and eye detection -- the ACF library is available as a standalone
   Hunter package `here <https://github.com/elucideye/acf>`__
-  Iris ellipse fitting via
   `Cascaded Pose Regression <https://pdollar.github.io/files/papers/DollarCVPR10pose.pdf>`__
   (Piotr Dollar, et al) + `XGBoost <https://github.com/dmlc/xgboost>`__
   regression (Tianqi Chen, et al)
-  Face landmarks and global eye models provided by
   `"One Millisecond Face Alignment with an Ensemble of Regression Trees <http://www.cvfoundation.org/openaccess/content_cvpr_2014/papers/Kazemi_One_Millisecond_Face_2014_CVPR_paper.pdf>`__
   (Kazemi, et al) using a modified implementation from
   `Dlib <https://github.com/davisking/dlib>`__ (Davis King)
   (normalized pixel differences, line indexed features, PCA size reductions)
-  OpenGL ES friendly GPGPU shader processing and efficient iOS +
   Android texture handling using a modified version of
   `ogles\_gpgpu <https://github.com/hunter-packages/ogles_gpgpu>`__
   (Markus Kondrad) with a number of shader implementations taken
   directly from `GPUImage <https://github.com/BradLarson/GPUImage>`__
   (Brad Larson)

+---------------------------+
| iPhone @ 30 FPS (VIDEO)   |
+===========================+
| |iPhone|                  |
+---------------------------+

Drishti Right Eye Annotation Scheme
-----------------------------------

+----------------+---------------------------------------------------------------+
| FEATURE        | SPECIFICATION                                                 |
+================+===============================================================+
| eyelids        | 2D points 0-15                                                |
+----------------+---------------------------------------------------------------+
| crease         | 2D points 16-24                                               |
+----------------+---------------------------------------------------------------+
| iris center    | 2D point 25                                                   |
+----------------+---------------------------------------------------------------+
| outer limbus   | limbus intersection with ray from outer corner to iris center |
+----------------+---------------------------------------------------------------+
| inner limbus   | limbus intersection with ray from inner corner to iris center |
+----------------+---------------------------------------------------------------+
| iris ellipse   | 2D center, minor axis, major axis, angle (radians)            |
+----------------+---------------------------------------------------------------+
| pupil ellipse  | 2D center, minor axis, major axis, angle (radians)            |
+----------------+---------------------------------------------------------------+

* the left eye is obtained by Y axis mirroring
* total (27*2)+(2*5) = 64 parameters
* the eye crease is useful for pose indexing, but better guidelines are needed
* the 2D limbus points are slightly redundant (given the ellipse iris model) but the intersection points are stable with respect to squinting and provide an efficient anchor for posed indexed features (accurate point-to-ellipse distances are non-trivial and are fairly computationally intensive)
* currently 2D only (gaze angle ground truth would be beneficial)

.. figure:: https://user-images.githubusercontent.com/554720/33522880-227e2468-d7c6-11e7-9705-13df5da04894.jpg
   :alt: drishti\_annotation\_scheme

Quick Start (i.e., How do I make this library work?)
----------------------------------------------------

General
~~~~~~~

Drishti is a `CMake <https://github.com/kitware/CMake>`__ based project
that uses the `Hunter <https://github.com/ruslo/hunter>`__ package
manager to download and build project dependencies from source as
needed. Hunter contains `detailed
documentation <https://docs.hunter.sh/en/latest>`__, but a few high
level notes and documentation links are provided here to help orient
first time users. In practice, some working knowledge of CMake may also
be required. Hunter itself is written in CMake, and is installed as part
of the build process from a single ``HunterGate()`` macro at the top of
the root ``CMakeLists.txt`` file (typically
``cmake/Hunter/HunterGate.cmake``) (you don't have to build or install
it). Each CMake dependency's ``find_package(FOO)`` call that is paired
with a ``hunter_add_package(FOO CONFIG REQUIRED)`` will be managed by
Hunter. In most cases, the only system requirement for building a Hunter
project is `a recent CMake <https://docs.hunter.sh/en/latest/quick-start/cmake.html>`__
, a working compiler corresponding to the operative toolchain and native build tool.
If you're not familiar with CMake, you can try to build
`this minimal example <https://cgold.readthedocs.io/en/latest/first-step.html>`__
to get a basic understanding.

Hunter will maintain all dependencies in a
`versioned <https://docs.hunter.sh/en/latest/overview/customization.html>`__
local
`cache <https://docs.hunter.sh/en/latest/overview/shareable.html>`__ by
default (typically ``${HOME}/.hunter``) where they can be reused in
subsequent builds and shared between different projects. They can also
be stored in a server side `binary
cache <https://docs.hunter.sh/en/latest/overview/binaries.html>`__ --
select `toolchains <#Toolchains>`__ will be backed by a server side
binary cache (https://github.com/elucideye/hunter-cache) and will
produce faster first time builds (use them if you can!).

Get Latest Sources
~~~~~~~~~~~~~~~~~~

Clone this repository and initialize all submodules:

.. code-block:: none

  > git clone https://github.com/elucideye/drishti
  > cd drishti
  [drishti]> git submodule update --init .

or

.. code-block:: none

  > git clone --recursive https://github.com/elucideye/drishti

Generate and Build
~~~~~~~~~~~~~~~~~~

Desktop platforms usually don't require a toolchain (a default toolchain with C++11 support will
be set by Drishti) and you can generate and build Drishti as a regular CMake project.

Linux + GCC + Makefile with Drishti examples, Release:

.. code-block:: none

  cmake -H. -B_builds -DHUNTER_STATUS_DEBUG=ON -DDRISHTI_BUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build _builds

macOS + Xcode with Drishti examples, Release:

.. code-block:: none

  cmake -H. -B_builds -GXcode -DHUNTER_STATUS_DEBUG=ON -DDRISHTI_BUILD_EXAMPLES=ON
  cmake --build _builds --config Release

Windows + Visual Studio 15 2017 with Drishti examples, Release:

.. code-block:: none

  cmake -H. -B_builds -G "Visual Studio 15 2017" -DHUNTER_STATUS_DEBUG=ON -DDRISHTI_BUILD_EXAMPLES=ON
  cmake --build _builds --config Release

To run the install procedure add the ``CMAKE_INSTALL_PREFIX`` variable
and use ``--target install``:

.. code-block:: none

  cmake -H. -B_builds -G "Visual Studio 15 2017" -DHUNTER_STATUS_DEBUG=ON -DCMAKE_INSTALL_PREFIX=_install
  cmake --build _builds --config Release --target install

Polly Based Build
~~~~~~~~~~~~~~~~~

To support cross platform builds and testing, the CI scripts make use of
`Polly <https://github.com/ruslo/polly>`__: a set of common CMake
toolchains paired with a simple ``polly.py`` CMake build script.
Polly is a Python script, make sure Python 3 is installed:

.. code-block:: none

  > which python3
  /usr/bin/python3

Clone Polly and add ``bin`` folder to ``PATH``:

.. code-block:: none

  > git clone https://github.com/ruslo/polly
  > export PATH=`pwd`/polly/bin:$PATH

Check it:

.. code-block:: none

  > which polly.py
  /.../polly/bin/polly.py

  > polly.py --help
  Python version: 3.5
  usage: polly.py [-h]
      [--toolchain ...

Note: Polly is not a build requirement, CMake can always be used
directly, but it is used here for convenience.

After the environment is configured, you can build for any supported
``Polly`` toolchain (below you can find some toolchains used in CI) with a command like this:

.. code-block:: bash

    polly.py --toolchain ${TOOLCHAIN} --config-all ${CONFIG} --install --verbose

Building examples:

.. code-block:: bash

    polly.py --toolchain ${TOOLCHAIN} --config-all ${CONFIG} --install --verbose --reconfig --fwd DRISHTI_BUILD_EXAMPLES=ON

::


Note: The ``--reconfig`` flag is included in the example above, which will
re-run the CMake configure step (to incorporate CMake changes) for you.  It is
a reasonable step to add in cases where you aren't sure if it is needed.

iOS Build
~~~~~~~~~

Since CMake contains an Xcode generator, building for ``iOS`` is fairly straightforward.
In practice, it is no different than the other `polly.py` toolchain builds.  As always,
you will need to have an Apple Developer Account to build and run on ``iOS`` devices.
There are a few setup steps associated with Apple code signing requirements.
Since iOS 10.0, Xcode projects require a valid `Team ID` entry,
which can be set through CMake using the `CMAKE_XCODE_ATTRIBUTE_DEVELOPMENT_TEAM` CMake variable.
If you generate an Xcode project through a `polly.py` command (described below), it will initialize
the field for you if the
`POLLY_IOS_DEVELOPMENT_TEAM <https://polly.readthedocs.io/en/latest/toolchains/ios/errors/polly_ios_development_team.html#polly-ios-development-team>`__
environment variable is set with your `Team ID`, which
can be found in your `Apple Developer Account <https://developer.apple.com/account/#/membership>`__.
If you are using an Apple Enterprise Developer Account, the ``CMAKE_TRY_COMPILE`` step can
fail with an error beginning with `No profiles for 'com.example' were found: ...`.
You can fix this with a one time Xcode initialization described in
`POLLY_IOS_DEVELOPMENT_TEAM <https://polly.readthedocs.io/en/latest/toolchains/ios/errors/polly_ios_bundle_identifier.html#polly-ios-bundle-identifier>`__.

Android Studio Build
~~~~~~~~~~~~~~~~~~~~

For Android Studio, there are additional requirements:

* CMake 3.9.2+
* Ninja
* Android Studio 3.2.1

Note: Polly will not be used here, because CMake is launched by Android Studio
itself.

Note: Host compiler is required for some parts of the build.
E.g. on Windows you have to install
`Visual Studio <https://cgold.readthedocs.io/en/latest/first-step/native-build-tool/visual-studio.html>`__.
Please check that `minimal C++ example <https://cgold.readthedocs.io/en/latest/first-step/minimal-example.html>`__
is working.

The path to the CMake executable should be added to the ``local.properties``
file before opening ``drishti/android-studio`` in Android Studio, or before
invoking the Gradle build script.  If you do not have a ``local.properties``
file, it will be generated automatically by Android Studio in the top level
``android-studio`` folder (usually ``drishti/android-studio/local.properites``),
when it is launched, at which point you can add the ``cmake.dir=/path/to/native/cmake``
CMake entry and rerun.  The ``local.properties`` file will look something like this:


.. code-block:: none

    ndk.dir=/home/username/Android/Sdk/ndk-bundle
    sdk.dir=/home/username/Android/Sdk
    cmake.dir=/opt/cmake

The ``cmake.dir`` entry should be set such that ``<cmake.dir>/bin/cmake`` points to a
valid ``cmake`` executable file.

Please check these instructions for details and useful notes:

* https://docs.hunter.sh/en/latest/faq/android-studio.html

There is another entry point for Android Studio - ``src/examples/facefilter/android-studio``.
It should be used only for testing or as a template for starting your own project
based on Drishti.

Android Studio Workarounds
~~~~~~~~~~~~~~~~~~~~~~~~~~

The following factors may contribute to some instability in the Android
Studio managed build.

* Using custom CMake 3.7+ in Android Studio is `a relatively new feature <https://developer.android.com/studio/projects/add-native-code#vanilla_cmake>`__
* Some issues are hard to track or confirm, some `issues <https://issuetracker.google.com/issues/75268076>`__ are already reported but still **not fixed**

With support for official CMake binaries now in the Android Studio 3.2.1 release,
the Android build stability is very likely improved compared to earlier beta versions.

From experience, the weakest part in the build has been communication between
Gradle and CMake. To minimize it, the following trick can be used:

* Open the top-level ``CMakeLists.txt`` file
* Find ``if(DRISHTI_DEBUG_STOP)`` `condition <https://github.com/elucideye/drishti/blob/d8b91e26eb1a1f62412bd2d56d1a229d646b6864/CMakeLists.txt#L102-L107>`__
* Substitute ``if(DRISHTI_DEBUG_STOP)`` with ``if(TRUE)``
* Run Gradle build:

.. code-block:: none

  [drishti]> cd android-studio
  [drishti/android-studio]> ./gradlew assembleDebug

If you're running it a first time there will be a high chance to hit this
Gradle issue:

.. code-block:: none

  * What went wrong:
  Execution failed for task '...'.
  > Conversion = c, Flags =

In this case, just wait for few seconds and run Gradle again:

.. code-block:: none

  [drishti/android-studio]> ./gradlew assembleDebug

* Revert ``CMakeLists.txt`` file, i.e. substitute ``if(TRUE)`` with ``if(DRISHTI_DEBUG_STOP)``.

* Run the CMake build without Gradle:

.. code-block:: none

  [drishti/android-studio]> cmake --build ../src/examples/facefilter/android-studio/app/.externalNativeBuild/cmake/debug/arm64-v8a

Once the CMake build is ready, you can use ``./gradlew assembleDebug`` or open
Android Studio IDE.

Applications
------------

Please see the README for the `drishti-hci <https://github.com/elucideye/drishti/blob/master/src/app/hci/README.rst>`__
console application to see an example of a full eye tracking pipeline with the GPGPU optimizations.

Integration
-----------

Drishti is also available as a Hunter package.  If you would like to integrate
Drishti in your project, please see the Hunter
`Drishti package documentation <https://docs.hunter.sh/en/latest/packages/pkg/drishti.html#pkg-drishti>`__.

Steps (check https://docs.hunter.sh/en/latest/quick-start.html):

Add ``cmake/HunterGate.cmake`` and a minimal ``cmake/Hunter/config.cmake`` to your project:

.. code-block:: cmake

    mkdir -p cmake/Hunter
    wget https://raw.githubusercontent.com/hunter-packages/gate/master/cmake/HunterGate.cmake -O cmake/HunterGate.cmake
    wget https://raw.githubusercontent.com/ruslo/hunter/master/examples/drishti/config.cmake -O cmake/Hunter/config.cmake

::

Add ``HunterGate(URL <url> SHA1 <sha1>)`` to the top of your ``CMakeLists.txt`` (You can find updated release information `here <https://github.com/ruslo/hunter/releases>`__).

.. code-block:: cmake

    include("cmake/HunterGate.cmake")
    HunterGate(
        URL "https://github.com/ruslo/hunter/archive/v0.19.140.tar.gz"
        SHA1 "f2c30348c05d0d424976648ce3560044e007496c"
        LOCAL # use cmake/Hunter/config.cmake
    )

::

Finally, add the Drishti package to your CMakeLists.txt and link it to your target:

.. code-block:: cmake

    hunter_add_package(drishti)
    find_package(drishti CONFIG REQUIRED)
    target_link_libraries(your_app_or_lib PUBLIC drishti::drishti)

::

You can customize the drishti package (and dependencies) by specifying a `VERSION` and/or `CMAKE_ARGS` (options) list for each package in ``cmake/Hunter/config.cmake``.

Please see https://github.com/elucideye/drishti_hunter_test for a minimal working example using the drishti hunter package.

Toolchains
----------

The configurations listed below have all been tested. In general, most
C++11 toolchains should work with minimal effort. A ``CI`` comment
indicates that the configuration is part of the Travis or Appveyor CI
tests, so all Hunter packages will be available in the server side
binary cache.

Linux (Ubunty Trusty 14.04):

* ``TOOLCHAIN=clang-fpic-hid-sections`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=gcc-5-pic-hid-sections-lto`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=libcxx`` ``CONFIG=Release`` # w/ clang 3.8

OSX:

* ``TOOLCHAIN=osx-10-13`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=osx-10-12-sanitize-address-hid-sections`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=xcode-hid-sections`` ``CONFIG=Release`` # generic

iOS:

* ``TOOLCHAIN=ios-nocodesign-11-3-dep-9-3-arm64`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=ios-10-1-arm64-dep-8-0-hid-sections`` ``CONFIG=Release``

Android:

* ``TOOLCHAIN=android-ndk-r17-api-19-armeabi-v7a-neon-clang-libcxx`` ``CONFIG=MinSizeRel`` # CI
* ``TOOLCHAIN=android-ndk-r17-api-24-arm64-v8a-clang-libcxx14`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections-lto`` ``CONFIG=MinSizeRel``

Windows:

* ``TOOLCHAIN=vs-15-2017`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=vs-14-2015-sdk-8-1`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=vs-14-2015-win64-sdk-8-1`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=vs-14-2015-win64-sdk-8-1`` ``CONFIG=Debug`` # CI

The polly out of source build trees are located in
``_builds/${TOOLCHAIN}``, the final build products (the stuff you want)
are installed in ``_install/${TOOLCHAIN}``, and the build logs are
dumped in ``_logs/${TOOLCHAIN}``. The iOS frameworks are installed in
``_frameworks/${TOOLCHAIN}``.

Choosing simplest toolchain
---------------------------

On Linux you will usually want ``--toolchain gcc-pic`` (GCC based toolchain with position independent code).

On Windows, the preferred toolchain will depend on the generator you want, e.g.,
if you want "Visual Studio 15 2017", then use ``--toolchain vs-15-2017``, if you
want the 64 bit version use ``--toolchain vs-15-2017-win64``.

On macOS, the choice of toolchain depends on Xcode version you have installed.
Please check this table for Xcode versions and corresponding iOS/macOS SDK
versions:

* https://polly.readthedocs.io/en/latest/toolchains/ios.html

E.g., if you have Xcode 8.3.1 installed, then the default SDK will be macOS
10.12 SDK, hence you can use ``--toolchain osx-10-12``. Instead of the Xcode
generator, you can use a Makefile toolchain - ``--toolchain osx-10-12-make``.

In the same table, you can find iOS SDK version. E.g., if you have installed
Xcode 9.4 with default iOS SDK 11.4, and you want to set the deployment SDK
to version 9.3, you can use ``--toolchain ios-11-4-dep-9-3-arm64`` to build
the ARM64 architecture.  If you have several versions of Xcode installed, you
can use ``IOS_X_Y_DEVELOPER_DIR``/``OSX_X_Y_DEVELOPER_DIR`` environment
variables for switching. E.g., if ``OSX_10_13_DEVELOPER_DIR`` will be set to
Xcode 9.0 location, then Xcode 9.0 will be used with ``--toolchain osx-10-13``,
even if Xcode 9.3 is installed and set as the default.

You can use Polly toolchains to build Android if you don't want to rely on
Android Studio. The only requirement is an environment variable with the
Android NDK location. Set the ``ANDROID_NDK_r17`` environment variable with
the path to the Android NDK r17, and you can use any
``--toolchain android-ndk-r17-*`` variants.

.. |Travis| image:: https://img.shields.io/travis/elucideye/drishti/master.svg?style=flat-square&label=Linux%20OSX%20Android%20iOS
   :target: https://travis-ci.org/elucideye/drishti/builds
.. |Appveyor| image:: https://img.shields.io/appveyor/ci/headupinclouds/drishti.svg?style=flat-square&label=Windows
   :target: https://ci.appveyor.com/project/headupinclouds/drishti
.. |License (3-Clause BSD)| image:: https://img.shields.io/badge/license-BSD%203--Clause-brightgreen.svg?style=flat-square
   :target: http://opensource.org/licenses/BSD-3-Clause
.. |Hunter| image:: https://img.shields.io/badge/hunter-drishti-blue.svg
   :target: https://docs.hunter.sh/en/latest/packages/pkg/drishti.html
.. |Gitter| image:: https://badges.gitter.im/elucideye/drishti.svg
   :target: https://gitter.im/elucideye/drishti?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge
.. |eye models 1| image:: https://user-images.githubusercontent.com/554720/28920911-d836e56a-7821-11e7-8b41-bc338f100cc1.png
.. |eye models 2| image:: https://user-images.githubusercontent.com/554720/28920912-da9f3820-7821-11e7-848c-f526922e24ec.png
.. |eye models 3| image:: https://user-images.githubusercontent.com/554720/28920920-dcd8e708-7821-11e7-8fc2-b9f375a9a550.png
.. |iPhone| image:: https://goo.gl/1uLQ44
   :target: https://vimeo.com/230351171

drishti
=======

|Travis| |Appveyor| |License (3-Clause BSD)| |Hunter| |Gitter|

.. figure:: https://user-images.githubusercontent.com/554720/28922218-3a005f9c-7827-11e7-839c-ef3e9a282f70.png
   :alt: drishti\_text\_big

Real time eye tracking for embedded and mobile devices in C++11.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|eye models 1| |eye models 2| |eye models 3|

Overview
--------

Goal: SDK size <= 1 MB and combined resources (object detection +
regression models) <= 4 MB.

-  `Hunter <https://github.com/ruslo/hunter>`__ package management and
   CMake build system by Ruslan Baratov, as well as much of the cross
   platform Qt work: "Organized Freedom!" :)
-  A C++ and OpenGL ES 2.0 implementation of `Fast Feature Pyramids for
   Object
   Detection <https://pdollar.github.io/files/papers/DollarPAMI14pyramids.pdf>`__
   (see `Piotr's Matlab Toolbox <https://pdollar.github.io/toolbox>`__)
   for face and eye detection; UPDATE: The acf code has been moved to a separate
   repository with a hunter package `HERE <https://github.com/elucideye/acf>`__
-  Iris ellipse fitting via `Cascaded Pose
   Regression <https://pdollar.github.io/files/papers/DollarCVPR10pose.pdf>`__
   (Piotr Dollar, et al) + `XGBoost <https://github.com/dmlc/xgboost>`__
   regression (Tianqi Chen, et al)
-  Face landmarks and eye contours provided by `"One Millisecond Face
   Alignment with an Ensemble of Regression
   Trees" <http://www.cv-foundation.org/openaccess/content_cvpr_2014/papers/Kazemi_One_Millisecond_Face_2014_CVPR_paper.pdf>`__
   (Kazemi, et al) using a modified implementation from
   `Dlib <https://github.com/davisking/dlib>`__ (Davis King) (normalized
   pixel differences, line indexed features, PCA size reductions)
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
project is a recent `CMake with
CURL <https://docs.hunter.sh/en/latest/contributing.html#reporting-bugs>`__
support and a working compiler correpsonding to the operative toolchain.
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

The
`Travis <https://github.com/elucideye/drishti/blob/master/.travis.yml>`__
(Linux/OSX/iOS/Android) and
`Appveyor <https://github.com/elucideye/drishti/blob/master/appveyor.yml>`__
(Windows) CI scripts in the project's root directory can serve as a
reference for basic setup when building from source. To support cross
platform builds and testing, the CI scripts make use of
`Polly <https://github.com/ruslo/polly>`__: a set of common CMake
toolchains paired with a simple ``polly.py`` CMake build script. Polly
is used here for convenience to generate ``CMake`` command line
invocations -- it is not required for building Hunter projects.

To reproduce the CI builds on a local host, the following setup is
recommended:

-  Install compiler:
   http://cgold.readthedocs.io/en/latest/first-step.html
-  Install `CMake <https://github.com/kitware/CMake>`__ (and add to
   ``PATH``)
-  Install Python (for Polly)
-  Clone `Polly <https://github.com/ruslo/polly>`__ and add
   ``<polly>/bin`` to ``PATH``

Note: Polly is not a build requirement, CMake can always be used
directly, but it is used here for convenience.

The ``bin/hunter_env.{sh,cmd}`` scripts (used in the CI builds) can be
used as a fast shortcut to install these tools for you. You may want to
add the ``PATH`` variables permanently to your ``.bashrc`` file (or
equivalent) for future sessions.

+--------------------------------+--------------------------+
| Linux/OSX/Android/iOS          | Windows                  |
+================================+==========================+
| ``source bin/hunter_env.sh``   | ``bin\hunter_env.cmd``   |
+--------------------------------+--------------------------+

After the environment is configured, you can build for any supported
``Polly`` toolchain (see ``polly.py --help``) with a command like this:

.. code-block:: bash

    polly.py --toolchain ${TOOLCHAIN} --config ${CONFIG} --fwd HUNTER_CONFIGURATION_TYPES=${CONFIG} --install --verbose
    
::

Applications
------------

Please see the README for the `drishti-hci <https://github.com/elucideye/drishti/blob/master/src/app/hci/README.rst>`__
console application to see an example of a full eye tracking pipeline with the GPGPU optimizations.


Integration
-----------

Drishti is also available as a hunter package.  If you would like to integrate drishti in your project, please see the hunter  `drishti package documentation <https://docs.hunter.sh/en/latest/packages/pkg/drishti.html#pkg-drishti>`__.

Steps:

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

Finally, add the drishti package to your CMakeLists.txt and link it to your target:

.. code-block:: cmake

    hunter_add_package(drishti)
    find_package(drishti CONFIG REQUIRED)
    target_link_libraries(your_app_or_lib drishti::drishti)

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

* ``TOOLCHAIN=gcc-5-pic-hid-sections-lto`` ``CONFIG=Release`` # CI 
* ``TOOLCHAIN=libcxx`` ``CONFIG=Release`` # w/ clang 3.8

OSX: 

* ``TOOLCHAIN=osx-10-11-hid-sections-lto`` ``CONFIG=Release`` # CI
* ``TOOLCHAIN=osx-10-12-sanitize-address-hid-sections`` ``CONFIG=Release`` # CI 
* ``TOOLCHAIN=xcode-hid-sections`` ``CONFIG=Release`` # generic

iOS: 

* ``TOOLCHAIN=ios-nocodesign-10-1-arm64-dep-9-0-device-libcxx-hid-sections-lto`` ``CONFIG=MinSizeRel`` # CI 
* ``TOOLCHAIN=ios-10-1-arm64-dep-8-0-hid-sections`` ``CONFIG=Release``

Android (from OSX): 

* ``TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections`` ``CONFIG=MinSizeRel`` # CI 
* ``TOOLCHAIN=android-ndk-r10e-api-19-armeabi-v7a-neon-hid-sections-lto`` ``CONFIG=MinSizeRel``

Windows: 

* ``TOOLCHAIN=vs-14-2015-sdk-8-1`` ``CONFIG=Release`` # CI 
* ``TOOLCHAIN=vs-14-2015-sdk-8-1`` ``CONFIG=Debug`` # CI 
* ``TOOLCHAIN=vs-14-2015-win64-sdk-8-1`` ``CONFIG=Release`` # CI 
* ``TOOLCHAIN=vs-14-2015-win64-sdk-8-1`` ``CONFIG=Debug`` # CI

The polly out of source build trees are located in
``_builds/${TOOLCHAIN}``, the final build products (the stuff you want)
are installed in ``_install/${TOOLCHAIN}``, and the build logs are
dumped in ``_logs/${TOOLCHAIN}``. The iOS frameworks are installed in
``_frameworks/${TOOLCHAIN}``.

.. |Travis| image:: https://img.shields.io/travis/elucideye/drishti/master.svg?style=flat-square&label=Linux%20OSX%20Android%20iOS
   :target: https://travis-ci.org/elucideye/drishti/builds
.. |Appveyor| image:: https://img.shields.io/appveyor/ci/headupinclouds/drishti.svg?style=flat-square&label=Windows
   :target: https://ci.appveyor.com/project/headupinclouds/drishti
.. |License (3-Clause BSD)| image:: https://img.shields.io/badge/license-BSD%203--Clause-brightgreen.svg?style=flat-square
   :target: http://opensource.org/licenses/BSD-3-Clause
.. |Hunter| image:: https://img.shields.io/badge/hunter-v0.19.94-blue.svg
   :target: http://github.com/ruslo/hunter
.. |Gitter| image:: https://badges.gitter.im/elucideye/drishti.svg
   :target: https://gitter.im/elucideye/drishti?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge
.. |eye models 1| image:: https://user-images.githubusercontent.com/554720/28920911-d836e56a-7821-11e7-8b41-bc338f100cc1.png
.. |eye models 2| image:: https://user-images.githubusercontent.com/554720/28920912-da9f3820-7821-11e7-848c-f526922e24ec.png
.. |eye models 3| image:: https://user-images.githubusercontent.com/554720/28920920-dcd8e708-7821-11e7-8fc2-b9f375a9a550.png
.. |iPhone| image:: https://goo.gl/1uLQ44
   :target: https://vimeo.com/230351171
   

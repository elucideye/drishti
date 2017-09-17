# drishti

[![Travis][travis_shield]][travis_builds]
[![Appveyor][appveyor_shield]][appveyor_builds]
[![License (3-Clause BSD)][license_shield]][license_link]
[![Hunter][hunter_shield]][hunter_link]

[hunter_shield]: https://img.shields.io/badge/hunter-v0.19.75-blue.svg
[hunter_link]: http://github.com/ruslo/hunter

[license_shield]: https://img.shields.io/badge/license-BSD%203--Clause-brightgreen.svg?style=flat-square
[license_link]: http://opensource.org/licenses/BSD-3-Clause

[travis_builds]: https://travis-ci.com/elucideye/drishti/builds
[travis_shield]: https://img.shields.io/travis/elucideye/drishti/master.svg?style=flat-square&label=Linux%20OSX%20Android%20iOS

[appveyor_builds]: https://ci.appveyor.com/api/projects/elucideye/drishti/history
[appveyor_shield]: https://img.shields.io/appveyor/ci/ingenue/hunter/pkg.gtest.svg?style=flat-square&label=Windows

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

/*!
  @file   FaceDetectorFactory.h
  @author David Hirvonen
  @brief  Internal declaration of a factory for FaceDetector modules

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__FaceDetectorFactory__
#define __DRISHTI__FaceDetectorFactory__

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"
#include <memory>
#include <string>
#include <vector>

// *INDENT-OFF*
namespace drishti { namespace ml { class ObjectDetector; } };
namespace drishti { namespace ml { class ShapeEstimator; } };
namespace drishti { namespace eye { class EyeModelEstimator; } };
// *INDENT-ON*

DRISHTI_FACE_NAMESPACE_BEGIN

struct FaceDetectorFactory
{
    FaceDetectorFactory() {}
    
    FaceDetectorFactory(
        const std::string &sFaceDetector,
        const std::vector<std::string> &sFaceRegressors,
        const std::string &sEyeRegressor,
        const std::string &sFaceDetectorMean)
    : sFaceDetector(sFaceDetector)
    , sFaceRegressors(sFaceRegressors)
    , sEyeRegressor(sEyeRegressor)
    , sFaceDetectorMean(sFaceDetectorMean)
    {}
    
    std::string sFaceDetector;
    std::vector<std::string> sFaceRegressors;
    std::string sEyeRegressor;
    std::string sFaceDetectorMean;
    
    virtual std::unique_ptr<drishti::ml::ObjectDetector> getFaceDetector();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getInnerFaceEstimator();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getOuterFaceEstimator();
    virtual std::unique_ptr<drishti::eye::EyeModelEstimator> getEyeEstimator();
    virtual drishti::face::FaceModel getMeanFace();
};

DRISHTI_FACE_NAMESPACE_END

#endif // __DRISHTI__FaceDetectorFactory__

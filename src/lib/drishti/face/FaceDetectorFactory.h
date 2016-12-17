/*!
  @file   FaceDetectorFactory.h
  @author David Hirvonen
  @brief  Internal declaration of a factory for FaceDetector modules

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceDetectorFactory_h__
#define __drishti_face_FaceDetectorFactory_h__

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

class FaceDetectorFactory
{
public:
    
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
    
    virtual std::unique_ptr<drishti::ml::ObjectDetector> getFaceDetector();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getInnerFaceEstimator();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getOuterFaceEstimator();
    virtual std::unique_ptr<drishti::eye::EyeModelEstimator> getEyeEstimator();
    virtual drishti::face::FaceModel getMeanFace();

    std::string sFaceDetector;
    std::vector<std::string> sFaceRegressors;
    std::string sEyeRegressor;
    std::string sFaceDetectorMean;
};

class FaceDetectorFactoryStream : public FaceDetectorFactory
{
public:

    FaceDetectorFactoryStream() {}
    
    FaceDetectorFactoryStream(
        std::istream *iFaceDetector,
        std::vector<std::istream*> &iFaceRegressors,
        std::istream *iEyeRegressor,
        std::istream *iFaceDetectorMean)
    : iFaceDetector(iFaceDetector)
    , iFaceRegressors(iFaceRegressors)
    , iEyeRegressor(iEyeRegressor)
    , iFaceDetectorMean(iFaceDetectorMean)
    {}
        
    virtual std::unique_ptr<drishti::ml::ObjectDetector> getFaceDetector();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getInnerFaceEstimator();
    virtual std::unique_ptr<drishti::ml::ShapeEstimator> getOuterFaceEstimator();
    virtual std::unique_ptr<drishti::eye::EyeModelEstimator> getEyeEstimator();
    virtual drishti::face::FaceModel getMeanFace();

    std::istream *iFaceDetector = nullptr;
    std::vector<std::istream *> iFaceRegressors;
    std::istream *iEyeRegressor = nullptr;
    std::istream *iFaceDetectorMean = nullptr;    
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceDetectorFactory_h__

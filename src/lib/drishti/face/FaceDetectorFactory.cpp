#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"
#include "drishti/acf/ACF.h" // ACF detection
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeModelEstimator.h"

#include <fstream>

DRISHTI_FACE_NAMESPACE_BEGIN

drishti::face::FaceModel loadFaceModel(std::istream &is);
drishti::face::FaceModel loadFaceModel(const std::string &filename);

/*
 * FaceDetectorFactor (string)
 */

std::unique_ptr<ml::ObjectDetector> FaceDetectorFactory::getFaceDetector()
{
    return core::make_unique<acf::Detector>(sFaceDetector);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactory::getInnerFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(sFaceRegressors[0]);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactory::getOuterFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(sFaceRegressors[1]);
}

std::unique_ptr<eye::EyeModelEstimator> FaceDetectorFactory::getEyeEstimator()
{
    return core::make_unique<eye::EyeModelEstimator>(sFaceDetector);
}

face::FaceModel FaceDetectorFactory::getMeanFace()
{
    face::FaceModel faceDetectorMean;    
    if(sFaceDetectorMean.empty())
    {
        faceDetectorMean = loadFaceModel(sFaceDetectorMean);
    }
    return faceDetectorMean;
}

/*
 * FaceDetectorFactorStream (std::istream)
 */

std::unique_ptr<ml::ObjectDetector> FaceDetectorFactoryStream::getFaceDetector()
{
    return core::make_unique<acf::Detector>(*iFaceDetector);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactoryStream::getInnerFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(*iFaceRegressors[0]);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactoryStream::getOuterFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(*iFaceRegressors[1]);
}

std::unique_ptr<eye::EyeModelEstimator> FaceDetectorFactoryStream::getEyeEstimator()
{
    return core::make_unique<DRISHTI_EYE::EyeModelEstimator>(*iFaceDetector, sFaceDetector);
}

face::FaceModel FaceDetectorFactoryStream::getMeanFace()
{
    face::FaceModel faceDetectorMean;
    if(iFaceDetectorMean && *iFaceDetectorMean)
    {
        faceDetectorMean = loadFaceModel(*iFaceDetectorMean);
    }
    return faceDetectorMean;
}

DRISHTI_FACE_NAMESPACE_END


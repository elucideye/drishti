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

std::unique_ptr<drishti::ml::ObjectDetector> FaceDetectorFactory::getFaceDetector()
{
    return drishti::core::make_unique<drishti::acf::Detector>(sFaceDetector);
}

std::unique_ptr<drishti::ml::ShapeEstimator> FaceDetectorFactory::getInnerFaceEstimator()
{
    return drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(sFaceRegressors[0]);
}

std::unique_ptr<drishti::ml::ShapeEstimator> FaceDetectorFactory::getOuterFaceEstimator()
{
    return drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(sFaceRegressors[1]);
}

std::unique_ptr<drishti::eye::EyeModelEstimator> FaceDetectorFactory::getEyeEstimator()
{
    std::unique_ptr<DRISHTI_EYE::EyeModelEstimator> regressor(new DRISHTI_EYE::EyeModelEstimator);

    assert(false);
    return regressor;
}

drishti::face::FaceModel FaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;    
    if(sFaceDetectorMean.empty())
    {
        faceDetectorMean = loadFaceModel(sFaceDetectorMean);
    }
    return faceDetectorMean;
}

/*
 * FaceDetectorFactorStream (std::istream)
 */

std::unique_ptr<drishti::ml::ObjectDetector> FaceDetectorFactoryStream::getFaceDetector()
{
    return drishti::core::make_unique<drishti::acf::Detector>(*iFaceDetector);
}

std::unique_ptr<drishti::ml::ShapeEstimator> FaceDetectorFactoryStream::getInnerFaceEstimator()
{
    return drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(*iFaceRegressors[0]);
}

std::unique_ptr<drishti::ml::ShapeEstimator> FaceDetectorFactoryStream::getOuterFaceEstimator()
{
    return drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(*iFaceRegressors[1]);
}

std::unique_ptr<drishti::eye::EyeModelEstimator> FaceDetectorFactoryStream::getEyeEstimator()
{
    std::unique_ptr<DRISHTI_EYE::EyeModelEstimator> regressor(new DRISHTI_EYE::EyeModelEstimator);
    assert(false);
    return regressor;
}

drishti::face::FaceModel FaceDetectorFactoryStream::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;
    if(iFaceDetectorMean && *iFaceDetectorMean)
    {
        faceDetectorMean = loadFaceModel(*iFaceDetectorMean);
    }
    return faceDetectorMean;
}

DRISHTI_FACE_NAMESPACE_END


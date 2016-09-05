#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/core/drishti_core.h"
#include "drishti/acf/ACF.h" // ACF detection
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeModelEstimator.h"

BEGIN_FACE_NAMESPACE

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
    load_pba_z(sEyeRegressor, *regressor);
    return regressor;
}

drishti::face::FaceModel FaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;
    std::ifstream is(sFaceDetectorMean);
    if(is)
    {
        cereal::XMLInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia >> faceDetectorMean;
    }
    return faceDetectorMean;
}

END_FACE_NAMESPACE


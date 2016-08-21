#include "face/FaceDetectorFactory.h"
#include "core/drishti_core.h"
#include "acf/ACF.h" // ACF detection
#include "ml/ShapeEstimator.h"
#include "ml/RegressionTreeEnsembleShapeEstimator.h"
#include "face/Face.h"
#include "eye/EyeModelEstimator.h"

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


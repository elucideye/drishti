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
#if DRISHTI_SERIALIZE_WITH_BOOST        
    load_pba_z(sEyeRegressor, *regressor);
#else
    assert(false); exit(-1);
#endif
    return regressor;
}

drishti::face::FaceModel FaceDetectorFactory::getMeanFace()
{
    drishti::face::FaceModel faceDetectorMean;
    std::ifstream is(sFaceDetectorMean);
    if(is)
    {
#if DRISHTI_CEREAL_XML_JSON
        cereal::XMLInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia >> faceDetectorMean;
#else
        std::cerr << "Skipping JSON archive" << std::endl;
#endif
    }
    return faceDetectorMean;
}

DRISHTI_FACE_NAMESPACE_END


#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/ml/ObjectDetectorACF.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"

#include <acf/ACF.h> // ACF detection

#include "drishti/core/infix_iterator.h"
#include <iterator>
#include <fstream>

DRISHTI_FACE_NAMESPACE_BEGIN

drishti::face::FaceModel loadFaceModel(std::istream& is);
drishti::face::FaceModel loadFaceModel(const std::string& filename);

/*
 * FaceDetectorFactor (string)
 */

std::unique_ptr<ml::ObjectDetector> FaceDetectorFactory::getFaceDetector()
{
    return core::make_unique<drishti::ml::ObjectDetectorACF>(sFaceDetector);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactory::getFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(sFaceRegressor);
}

std::unique_ptr<eye::EyeModelEstimator> FaceDetectorFactory::getEyeEstimator()
{
    return core::make_unique<eye::EyeModelEstimator>(sEyeRegressor);
}

face::FaceModel FaceDetectorFactory::getMeanFace()
{
    face::FaceModel faceDetectorMean;
    if (!sFaceDetectorMean.empty())
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
    return drishti::core::make_unique<ml::ObjectDetectorACF>(*iFaceDetector);
}

std::unique_ptr<ml::ShapeEstimator> FaceDetectorFactoryStream::getFaceEstimator()
{
    return core::make_unique<ml::RegressionTreeEnsembleShapeEstimator>(*iFaceRegressor);
}

std::unique_ptr<eye::EyeModelEstimator> FaceDetectorFactoryStream::getEyeEstimator()
{
    iEyeRegressor->clear();
    iEyeRegressor->seekg(0, std::ios::beg);
    return core::make_unique<eye::EyeModelEstimator>(*iEyeRegressor, sEyeRegressor);
}

face::FaceModel FaceDetectorFactoryStream::getMeanFace()
{
    face::FaceModel faceDetectorMean;
    if (iFaceDetectorMean && *iFaceDetectorMean)
    {
        faceDetectorMean = loadFaceModel(*iFaceDetectorMean);
    }
    return faceDetectorMean;
}

/*
 * Utility
 */

std::ostream& operator<<(std::ostream& os, const FaceDetectorFactory& factory)
{
    os // standardized output for active face and eye models
        << "{\n"
        << "  faceDetector " << factory.sFaceDetector << "\n"
        << "  faceRegressor " << factory.sFaceRegressor << "\n"
        << "  eyeRegressor " << factory.sEyeRegressor << "\n"
        << "  faceDetectorMean " << factory.sFaceDetectorMean << "\n"
        << "}";

    return os;
}

DRISHTI_FACE_NAMESPACE_END

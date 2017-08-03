#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/acf/ACF.h" // ACF detection
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"

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

struct OptionalList
{
    OptionalList(const std::vector<std::string>& list)
        : list(list)
    {
    }
    const std::vector<std::string>& list;
};

std::ostream& operator<<(std::ostream& os, const OptionalList& opt)
{
    std::copy(opt.list.begin(), opt.list.end(), infix_ostream_iterator<std::string>(os, ","));
    return os;
}

std::ostream& operator<<(std::ostream& os, const FaceDetectorFactory& factory)
{
    os // standardized output for active face and eye models
        << "{\n"
        << "  faceDetector " << factory.sFaceDetector << "\n"
        << "  faceRegressor " << OptionalList(factory.sFaceRegressors) << "\n"
        << "  eyeRegressor " << factory.sEyeRegressor << "\n"
        << "  faceDetectorMean " << factory.sFaceDetectorMean << "\n"
        << "}";

    return os;
}

DRISHTI_FACE_NAMESPACE_END
